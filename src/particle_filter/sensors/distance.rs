use alloc::rc::Rc;
use core::{cell::RefCell, f32::consts::PI, time::Duration};

use nalgebra::{Matrix1xX, Matrix2, Matrix2xX, Matrix3xX, RowDVector, Vector2, Vector3};
use vexide::{float::Float, prelude::DistanceSensor, time::Instant};

use super::ParticleFilterSensor;
use crate::utils::{
    math::lerp,
    samplers::{multivariate_gaussian_sampler::GaussianSampler, Sampler},
    FIELD_WALL,
};

/// A struct containing shared data between LiDARs.
///
/// Multiple LiDARs can share data that is computed every epoch.
/// If for example there are eight LiDARs and four of them use the same
/// global data and the other four use a different one, then make two
/// [`LiDARPrecomputedData`]s.
pub struct LiDARPrecomputedData {
    precomputed: bool,
    transformed_heading_cosines: RowDVector<f32>,
    transformed_heading_sines: RowDVector<f32>,
}

impl LiDARPrecomputedData {
    pub fn new() -> Self {
        Self {
            precomputed: false,
            transformed_heading_cosines: RowDVector::zeros(0),
            transformed_heading_sines: RowDVector::zeros(0),
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
    global_angles: RowDVector<f32>,
    precompute_data: Rc<RefCell<LiDARPrecomputedData>>,

    /// The amount to scale the LiDAR reading by.
    /// This is in case your mounting is not perfectly straight,
    /// and this is the cosine value of the angular offset.
    scalar: f32,

    /// Distance sensors update at approximately 30hz≈30ms/each.
    /// This is to store the last time it was polled at.
    last_updated: Option<Instant>,
}
impl LiDAR {
    pub fn new(
        sensor_offset: Vector3<f32>,
        sensor_noise_covar_matrix: Matrix2<f32>,
        min_std_dev: f32,
        max_std_dev: f32,
        distance_sensor: Rc<DistanceSensor>,
        precompute_data: Rc<RefCell<LiDARPrecomputedData>>,
        scalar: Option<f32>,
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
            global_angles: RowDVector::zeros(1),
            precompute_data,
            scalar: scalar.unwrap_or(1.0),
            last_updated: None,
        }
    }
}

impl ParticleFilterSensor<3> for LiDAR {
    // TODO: logic simplication since we have timing checks.
    fn precompute(&mut self, positions: &Matrix3xX<f32>) {
        if let Some(last_updated) = self.last_updated {
            if last_updated.elapsed() < Duration::from_millis(30) {
                return;
            }
        }

        let mut precompute_data = self.precompute_data.borrow_mut();
        if !precompute_data.precomputed {
            self.last_updated = Some(Instant::now());
            precompute_data.transformed_heading_cosines = positions.row(2).map(|x| x.sin());
            precompute_data.transformed_heading_sines = positions.row(2).map(|x| -x.cos());
            precompute_data.precomputed = true;
        }
    }

    fn cleanup_precomputed(&mut self) {
        self.precompute_data.borrow_mut().precomputed = false;
    }

    fn update(&mut self, positions: &Matrix3xX<f32>, weights: &mut RowDVector<f32>) {
        // It returns false only if the precomputes were not computed,
        // specifically for when it has not been 30 ms.
        if !self.precompute_data.borrow().precomputed {
            return;
        }
        let detected_object = self.distance_sensor.object().unwrap_or(None);

        if let Some(detected_distance_mm) = detected_object.as_ref().map(|object| object.distance) {
            let detected_distance_mm = detected_distance_mm as f32 * self.scalar;
            self.global_angles = positions.row(2).map(|x| x + self.sensor_offset[2]);
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
                    .transformed_heading_cosines
                    .dot(&self.sensor_offset.fixed_rows::<1>(0))
                    - precompute_data
                        .transformed_heading_sines
                        .dot(&self.sensor_offset.fixed_rows::<1>(1)),
            );
            self.transformed_sensor_offsets.row_mut(1).add_scalar_mut(
                precompute_data
                    .transformed_heading_sines
                    .dot(&self.sensor_offset.fixed_rows::<1>(0))
                    + precompute_data
                        .transformed_heading_cosines
                        .dot(&self.sensor_offset.fixed_rows::<1>(1)),
            );

            let pos_array = positions.fixed_rows::<2>(0) + &self.transformed_sensor_offsets;
            let detected_distance = detected_distance_mm * 0.03937008;
            let x_divisors = self.sensor_unit_vectors.row(0);
            let y_divisors = self.sensor_unit_vectors.row(1);
            let x_error_1 = (pos_array.row(0).map(|x| FIELD_WALL - x))
                .component_div(&x_divisors)
                .map(|element| (element - detected_distance).abs()); // Right wall
            let x_error_2 = (pos_array.row(0).map(|x| -FIELD_WALL - x))
                .component_div(&x_divisors)
                .map(|element| (element - detected_distance).abs()); // Left wall
            let y_error_1 = (pos_array.row(1).map(|y| FIELD_WALL - y))
                .component_div(&y_divisors)
                .map(|element| (element - detected_distance).abs()); // Top wall
            let y_error_2 = (pos_array.row(1).map(|y| -FIELD_WALL - y))
                .component_div(&y_divisors)
                .map(|element| (element - detected_distance).abs()); // Bottom wall

            let x_errors: Matrix1xX<f32> = x_error_1.zip_map(&x_error_2, |a, b| a.min(b));
            let y_errors: Matrix1xX<f32> = y_error_1.zip_map(&y_error_2, |a, b| a.min(b));

            let mut errors = Matrix1xX::zeros(positions.ncols());
            for i in 0..positions.ncols() {
                let x = x_errors[i];
                let y = y_errors[i];
                errors[i] = if x < y {
                    x * self.sensor_unit_vectors.column(i).x.abs()
                } else {
                    y * self.sensor_unit_vectors.column(i).y.abs()
                };
            }

            let final_std_dev = if detected_distance_mm > 200.0 {
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
            apply_normal_pdf_row_vector(
                &mut errors,
                0.0,
                final_std_dev,
                Some(positions.ncols() as f32),
            );
            *weights *= errors
        }
    }
}

fn apply_normal_pdf_row_vector(
    errors: &mut Matrix1xX<f32>,
    mean: f32,
    std_dev: f32,
    scalar: Option<f32>,
) {
    let variance = std_dev * std_dev;
    let inv_two_variance = -0.5 / variance; // Precompute inverse for efficiency
    let scale = (1.0 / (2.0 * PI * variance).sqrt()) * scalar.unwrap_or(1.0);

    errors.apply(|x| {
        *x = scale * ((*x - mean).powi(2) * inv_two_variance).exp();
    });
}
