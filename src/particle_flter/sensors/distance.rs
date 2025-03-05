use alloc::rc::Rc;
use core::{cell::RefCell, f32::consts::PI};

use nalgebra::{DVector, Matrix1xX, Matrix2, Matrix2xX, Matrix3xX, RowDVector, Vector2, Vector3};
use vexide::prelude::{DistanceSensor, Float};

use super::ParticleFilterSensor;
use crate::utils::{
    math::lerp,
    samplers::{multivariate_gaussian_sampler::GaussianSampler, AbstractSampler},
};

pub struct LiDARPrecomputedData {
    precomputed: bool,
    precomputed_robot_cosines_rotated: RowDVector<f32>,
    precomputed_robot_sines_rotated: RowDVector<f32>,
}

impl LiDARPrecomputedData {
    pub fn new() -> Self {
        Self {
            precomputed: false,
            precomputed_robot_cosines_rotated: RowDVector::zeros(0),
            precomputed_robot_sines_rotated: RowDVector::zeros(0),
        }
    }
}

impl Default for LiDARPrecomputedData {
    fn default() -> Self {
        Self::new()
    }
}

pub struct LiDAR {
    distance_sensor: Rc<DistanceSensor>,
    /// Offset: the vector from the tracking center to the sensor position.
    /// Offset where (+, 0) refers to the right side and (0, +) refers to the front.
    /// Angle (z) is independent of (x, y).
    /// Measured counterclockwise with the angle facing forward being 0°.
    sensor_offset: Vector3<f32>,
    min_std_dev: f32,
    max_std_dev: f32,
    // Run two dimensional vector here to incorporate positional noise.
    sampler: Rc<RefCell<GaussianSampler<2, 5000>>>,
    sensor_unit_vectors: Matrix2xX<f32>,
    transformed_sensor_offsets: Matrix2xX<f32>,
    global_angles: DVector<f32>,
    precompute_data: Rc<RefCell<LiDARPrecomputedData>>,
}
impl LiDAR {
    pub fn new(
        sensor_offset: Vector3<f32>,
        sensor_noise_covar_matrix: Matrix2<f32>,
        min_std_dev: f32,
        max_std_dev: f32,
        distance_sensor: Rc<DistanceSensor>,
        precompute_data: Rc<RefCell<LiDARPrecomputedData>>,
    ) -> Self {
        let sampler = GaussianSampler::new(Vector2::zeros(), sensor_noise_covar_matrix);

        LiDAR {
            distance_sensor,
            sensor_offset,
            min_std_dev,
            max_std_dev,
            sampler: Rc::new(RefCell::new(sampler)),
            sensor_unit_vectors: Matrix2xX::zeros(2),
            transformed_sensor_offsets: Matrix2xX::zeros(0),
            global_angles: DVector::zeros(1),
            precompute_data,
        }
    }
}

// #[async_trait::async_trait(?Send)]
impl ParticleFilterSensor<3> for LiDAR {
    fn precompute(&mut self, positions: &Matrix3xX<f32>) {
        let mut precompute_data = self.precompute_data.borrow_mut();
        if !precompute_data.precomputed {
            precompute_data.precomputed_robot_cosines_rotated = positions.row(2).map(|x| x.sin());
            precompute_data.precomputed_robot_sines_rotated = positions.row(2).map(|x| -x.cos());
            precompute_data.precomputed = true;
        }
    }

    fn cleanup_precomputed(&mut self) {
        self.precompute_data.borrow_mut().precomputed = false;
    }

    fn update(&mut self, positions: &Matrix3xX<f32>, weights: &mut DVector<f32>) {
        let detected_object = self.distance_sensor.object().unwrap_or_default();

        if let Some(detected_distance_mm) = detected_object.as_ref().map(|object| object.distance) {
            let robot_thetas = positions.row(2).transpose();
            self.global_angles = robot_thetas.map(|x| x + self.sensor_offset[2]);
            self.sensor_unit_vectors
                .row_mut(0)
                .copy_from(&self.global_angles.map(|x| x.cos()));
            self.sensor_unit_vectors
                .row_mut(1)
                .copy_from(&self.global_angles.map(|x| x.sin()));

            let mut sampler = self.sampler.borrow_mut();
            self.transformed_sensor_offsets = sampler.sample_batch(positions.ncols());
            let precompute_data = self.precompute_data.borrow();
            self.transformed_sensor_offsets.row_mut(0).add_scalar_mut(
                precompute_data
                    .precomputed_robot_cosines_rotated
                    .dot(&self.sensor_offset.fixed_rows::<1>(0))
                    - precompute_data
                        .precomputed_robot_sines_rotated
                        .dot(&self.sensor_offset.fixed_rows::<1>(1)),
            );
            self.transformed_sensor_offsets.row_mut(1).add_scalar_mut(
                precompute_data
                    .precomputed_robot_sines_rotated
                    .dot(&self.sensor_offset.fixed_rows::<1>(0))
                    + precompute_data
                        .precomputed_robot_cosines_rotated
                        .dot(&self.sensor_offset.fixed_rows::<1>(1)),
            );

            let pos_array = positions.fixed_rows::<2>(0) + &self.transformed_sensor_offsets;

            let x_divisors = self.sensor_unit_vectors.row(0);
            let y_divisors = self.sensor_unit_vectors.row(1);
            let tx1 = (pos_array.row(0).map(|x| FIELD_WALL - x)).component_div(&x_divisors);
            let tx2 = (pos_array.row(0).map(|x| -FIELD_WALL - x)).component_div(&x_divisors);
            let ty1 = (pos_array.row(1).map(|y| FIELD_WALL - y)).component_div(&y_divisors);
            let ty2 = (pos_array.row(1).map(|y| -FIELD_WALL - y)).component_div(&y_divisors);

            let shortest: Matrix1xX<f32> = tx1
                .map(|element| {
                    if element > 0.0 {
                        element
                    } else {
                        f32::INFINITY
                    }
                })
                .zip_map(
                    &tx2.map(|element| {
                        if element > 0.0 {
                            element
                        } else {
                            f32::INFINITY
                        }
                    }),
                    |a, b| a.min(b),
                )
                .zip_map(
                    &ty1.map(|element| {
                        if element > 0.0 {
                            element
                        } else {
                            f32::INFINITY
                        }
                    }),
                    |a, b| a.min(b),
                )
                .zip_map(
                    &ty2.map(|element| {
                        if element > 0.0 {
                            element
                        } else {
                            f32::INFINITY
                        }
                    }),
                    |a, b| a.min(b),
                );

            let detected_distance = detected_distance_mm as f32 * 0.03937008;
            let errors = shortest.map(|distance| distance - detected_distance);
            let final_std_dev = if detected_distance_mm > 200 {
                lerp!(
                    self.max_std_dev,
                    self.min_std_dev,
                    match detected_object {
                        Some(object) => object.confidence as f32,
                        None => 0.0,
                    }
                )
            } else {
                0.5
            };

            *weights *= normal_pdf_row_vector(&errors, 0.0, final_std_dev).transpose();
        }
    }
}

fn normal_pdf_row_vector(errors: &Matrix1xX<f32>, mean: f32, std_dev: f32) -> Matrix1xX<f32> {
    let variance = std_dev * std_dev;
    let scale = 1.0 / (2.0 * PI * variance).sqrt();
    errors.map(|x| scale * (-0.5 * (x - mean).powi(2) / variance).exp())
}

const FIELD_WALL: f32 = 100.0; // Example value, replace with actual field wall distance
