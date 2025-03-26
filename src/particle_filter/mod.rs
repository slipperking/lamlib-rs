pub mod sensors;
use alloc::{rc::Rc, vec::Vec};
use core::{
    cell::RefCell,
    f32::{self},
    ops::AddAssign,
};

use nalgebra::{Matrix, Matrix2xX, Matrix3, Matrix3xX, RowDVector, Vector3};
use rand::{
    distr::{Distribution, Uniform},
    Rng,
};
use sensors::ParticleFilterSensor;
use veranda::SystemRng;

use crate::utils::samplers::{multivariate_gaussian_sampler::GaussianSampler, BatchSampler};
pub struct ParticleFilter {
    enabled: bool,
    estimate_position: Vector3<f32>,
    positions: Matrix3xX<f32>,
    new_positions: Matrix3xX<f32>,
    weights: RowDVector<f32>,
    sampler: GaussianSampler<3, 20000>,
    particle_count: usize,
    generator: SystemRng,
}

impl ParticleFilter {
    pub fn new(particle_count: usize, covariance_matrix: Matrix3<f32>) -> Self {
        let positions = Matrix3xX::from_element(particle_count, 0.0);
        let new_positions = Matrix3xX::from_element(particle_count, 0.0);
        let weights = RowDVector::from_element(particle_count, 1.0 / particle_count as f32);
        let sampler = GaussianSampler::new(Vector3::zeros(), covariance_matrix);

        ParticleFilter {
            enabled: false,
            estimate_position: Vector3::zeros(),
            positions,
            new_positions,
            weights,
            sampler,
            particle_count,
            generator: SystemRng::new(),
        }
    }

    pub fn set_filter_state(&mut self, state: bool, position: Option<Vector3<f32>>) {
        self.enabled = state;
        if state {
            if let Some(pos) = position {
                self.scatter_particles(&pos, 2.0);
            }
        }
    }

    pub fn filter_state(&self) -> bool {
        self.enabled
    }

    pub fn predict(&mut self, delta_odometry: &Vector3<f32>) {
        let noise = { self.sampler.sample_batch(self.particle_count) };

        self.positions += *delta_odometry + noise;
    }

    pub fn update(&mut self, sensors: &Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>) {
        for sensor in sensors.iter() {
            let mut sensor_mut = sensor.borrow_mut();
            sensor_mut.precompute(&self.positions);
            sensor_mut.update(&self.positions, &mut self.weights);
            self.normalize_weights();
        }

        for sensor in sensors.iter() {
            sensor.borrow_mut().cleanup_precomputed();
        }
    }

    pub fn resample(&mut self) {
        let sum = self.weights.iter().map(|&w| w * w).sum::<f32>();
        if sum <= 0.0 {
            Matrix::fill(&mut self.weights, 1.0 / self.particle_count as f32);
            return;
        }

        let ess = 1.0 / sum;
        if ess >= 0.5 * self.particle_count as f32 {
            return;
        }

        let mut partial_sums = self.weights.clone();
        for i in 1..self.particle_count {
            partial_sums[i] += partial_sums[i - 1];
        }

        let step = 1.0 / self.particle_count as f32;
        let dist = Uniform::new(0.0, step).expect("Failed to create uniform distribution.");
        let offset = dist.sample(&mut self.generator);

        for i in 0..self.particle_count {
            let target = offset + i as f32 * step;
            let idx = partial_sums
                .iter()
                .position(|&x| x >= target)
                .unwrap_or(self.particle_count - 1);
            self.new_positions
                .column_mut(i)
                .copy_from(&self.positions.column(idx));
        }

        core::mem::swap(&mut self.positions, &mut self.new_positions);
        Matrix::fill(&mut self.weights, 1.0 / self.particle_count as f32);
    }

    pub fn calculate_mean(&mut self) {
        self.estimate_position = &self.positions * &self.weights.transpose();
    }

    pub fn scatter_particles(&mut self, center: &Vector3<f32>, distance: f32) {
        let noise = {
            Matrix2xX::from_fn(self.particle_count, |_, _| {
                self.generator.random_range(-distance..distance)
            })
        };

        for i in 0..self.positions.ncols() {
            self.positions.set_column(i, center);
        }
        self.positions.fixed_rows_mut::<2>(0).add_assign(&noise);
        Matrix::fill(&mut self.weights, 1.0 / self.particle_count as f32);
    }

    fn normalize_weights(&mut self) {
        let sum = self.weights.sum() + f32::MIN_POSITIVE * self.particle_count as f32;
        self.weights = (self.weights.map(|x| x + f32::MIN_POSITIVE)) / sum;
    }

    pub fn run_filter(
        &mut self,
        delta_odometry: Vector3<f32>,
        sensors: &Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>,
    ) -> Option<nalgebra::Vector3<f32>> {
        if self.enabled {
            self.predict(&delta_odometry);
            self.update(sensors);
            self.resample();
            self.calculate_mean();
            Some(self.estimate_position)
        } else {
            None
        }
    }
}
