pub mod sensors;
use alloc::{rc::Rc, vec::Vec};
use core::{
    cell::RefCell,
    f32::{self},
    ops::{AddAssign, Deref, DerefMut},
};

use nalgebra::{Matrix, Matrix2xX, Matrix3, Matrix3xX, RowDVector, Vector3};
use rand::{
    distr::{Distribution, Uniform},
    Rng,
};
use sensors::ParticleFilterSensor;
use veranda::SystemRng;

use crate::utils::samplers::{multivariate_gaussian_sampler::GaussianSampler, Sampler};
pub struct ParticleFilter {
    enabled: bool,
    estimate_position: Rc<RefCell<Vector3<f32>>>,
    positions: Rc<RefCell<Matrix3xX<f32>>>,
    new_positions: RefCell<Matrix3xX<f32>>,
    weights: Rc<RefCell<RowDVector<f32>>>,
    sampler: RefCell<GaussianSampler<3, 20000>>,
    particle_count: usize,
    generator: Rc<RefCell<SystemRng>>,
}

impl ParticleFilter {
    pub fn new(particle_count: usize, covariance_matrix: Matrix3<f32>) -> Self {
        let positions = Matrix3xX::from_element(particle_count, 0.0);
        let new_positions = Matrix3xX::from_element(particle_count, 0.0);
        let weights = RowDVector::from_element(particle_count, 1.0 / particle_count as f32);
        let sampler = GaussianSampler::new(Vector3::zeros(), covariance_matrix);

        ParticleFilter {
            enabled: false,
            estimate_position: Rc::new(RefCell::new(Vector3::zeros())),
            positions: Rc::new(RefCell::new(positions)),
            new_positions: RefCell::new(new_positions),
            weights: Rc::new(RefCell::new(weights)),
            sampler: RefCell::new(sampler),
            particle_count,
            generator: Rc::new(RefCell::new(SystemRng::new())),
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

    pub fn predict(&self, delta_odometry: &Vector3<f32>) {
        let mut positions = self.positions.borrow_mut();
        let noise = { self.sampler.borrow_mut().sample_batch(self.particle_count) };

        *positions += *delta_odometry;
        *positions += noise;
    }

    pub fn update(&self, sensors: Rc<Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>>) {
        for sensor in sensors.iter() {
            let mut sensor_mut = sensor.borrow_mut();
            let positions = self.positions.borrow_mut();
            sensor_mut.precompute(&positions);
            sensor_mut.update(&positions, &mut self.weights.borrow_mut());
            drop(positions);
            self.normalize_weights();
        }

        for sensor in sensors.iter() {
            sensor.borrow_mut().cleanup_precomputed();
        }
    }

    pub fn resample(&self) {
        let mut positions = self.positions.borrow_mut();
        let mut weights = self.weights.borrow_mut();

        let sum = weights.iter().map(|&w| w * w).sum::<f32>();
        if sum <= 0.0 {
            Matrix::fill(weights.deref_mut(), 1.0 / self.particle_count as f32);
            return;
        }

        let ess = 1.0 / sum;
        if ess >= 0.5 * self.particle_count as f32 {
            return;
        }

        let mut partial_sums = weights.clone();
        for i in 1..self.particle_count {
            partial_sums[i] += partial_sums[i - 1];
        }

        let step = 1.0 / self.particle_count as f32;
        let dist = Uniform::new(0.0, step).expect("Failed to create uniform distribution.");
        let mut rng = self.generator.borrow_mut();
        let offset = dist.sample(&mut *rng);

        let mut new_positions = self.new_positions.borrow_mut();
        for i in 0..self.particle_count {
            let target = offset + i as f32 * step;
            let idx = partial_sums
                .iter()
                .position(|&x| x >= target)
                .unwrap_or(self.particle_count - 1);
            new_positions
                .column_mut(i)
                .copy_from(&positions.column(idx));
        }

        core::mem::swap(&mut *positions, &mut *new_positions);
        Matrix::fill(weights.deref_mut(), 1.0 / self.particle_count as f32);
    }

    pub fn calculate_mean(&self) {
        let positions = self.positions.borrow_mut();
        let weights = self.weights.borrow_mut();
        let mut estimate_position = self.estimate_position.borrow_mut();

        *estimate_position = positions.deref() * weights.deref().transpose();
    }

    pub fn scatter_particles(&self, center: &Vector3<f32>, distance: f32) {
        let noise = {
            let mut rng = self.generator.borrow_mut();
            Matrix2xX::from_fn(self.particle_count, |_, _| {
                rng.random_range(-distance..distance)
            })
        };

        let mut positions = self.positions.borrow_mut();

        for i in 0..positions.ncols() {
            positions.set_column(i, center);
        }
        positions.fixed_rows_mut::<2>(0).add_assign(&noise);
        core::mem::drop(positions);
        let mut weights = self.weights.borrow_mut();
        Matrix::fill(weights.deref_mut(), 1.0 / self.particle_count as f32);
    }

    fn normalize_weights(&self) {
        let mut weights = self.weights.borrow_mut();
        let sum = weights.sum() + f32::MIN_POSITIVE * self.particle_count as f32;
        *weights = (weights.clone().map(|x| x + f32::MIN_POSITIVE)) / sum;
    }

    pub fn run_filter(
        &self,
        delta_odometry: Vector3<f32>,
        sensors: Rc<Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>>,
    ) -> Option<nalgebra::Vector3<f32>> {
        if self.enabled {
            self.predict(&delta_odometry);
            self.update(sensors.clone());
            self.resample();
            self.calculate_mean();
            Some(*self.estimate_position.borrow())
        } else {
            None
        }
    }
}
