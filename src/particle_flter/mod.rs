pub mod sensors;
use alloc::{rc::Rc, vec::Vec};
use core::{
    cell::RefCell,
    f32::{self},
    ops::{Deref, DerefMut},
};

use nalgebra::{Matrix, Matrix2xX, Matrix3, Matrix3xX, RowDVector, Vector3};
use rand::{
    distr::{Distribution, Uniform},
    Rng,
};
use sensors::ParticleFilterSensor;
use veranda::SystemRng;
use vexide::core::sync::Mutex;

use crate::utils::samplers::{multivariate_gaussian_sampler::GaussianSampler, AbstractSampler};
pub struct ParticleFilter {
    enabled: bool,
    estimate_position: Rc<Mutex<Vector3<f32>>>,
    positions: Rc<Mutex<Matrix3xX<f32>>>,
    new_positions: Rc<Mutex<Matrix3xX<f32>>>,
    weights: Rc<Mutex<RowDVector<f32>>>,
    sampler: Rc<Mutex<GaussianSampler<3, 20000>>>,
    particle_count: usize,
    generator: Rc<Mutex<SystemRng>>,
}

impl ParticleFilter {
    pub fn new(particle_count: usize, covariance_matrix: Matrix3<f32>) -> Self {
        let positions = Matrix3xX::from_element(particle_count, 0.0);
        let new_positions = Matrix3xX::from_element(particle_count, 0.0);
        let weights = RowDVector::from_element(particle_count, 1.0 / particle_count as f32);
        let sampler = GaussianSampler::new(Vector3::zeros(), covariance_matrix);

        ParticleFilter {
            enabled: false,
            estimate_position: Rc::new(Mutex::new(Vector3::zeros())),
            positions: Rc::new(Mutex::new(positions)),
            new_positions: Rc::new(Mutex::new(new_positions)),
            weights: Rc::new(Mutex::new(weights)),
            sampler: Rc::new(Mutex::new(sampler)),
            particle_count,
            generator: Rc::new(Mutex::new(SystemRng::new())),
        }
    }

    pub async fn set_filter_state(&mut self, state: bool, position: Option<Vector3<f32>>) {
        self.enabled = state;
        if state {
            if let Some(pos) = position {
                self.scatter_particles(&pos, 2.0).await;
            }
        }
    }

    pub async fn filter_state(&self) -> bool {
        self.enabled
    }

    pub async fn predict(&self, delta_odometry: &Vector3<f32>) {
        let mut positions = self.positions.lock().await;
        let noise = { self.sampler.lock().await.sample_batch(self.particle_count) };

        *positions += *delta_odometry;
        *positions += noise;
    }

    pub async fn update(&self, sensors: Rc<Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>>) {
        {
            let positions = self.positions.lock().await;
            let mut weights = self.weights.lock().await;

            for sensor in sensors.iter() {
                let mut sensor_mut = sensor.borrow_mut();
                sensor_mut.precompute(&positions);
                sensor_mut.update(&positions, &mut weights);
            }

            for sensor in sensors.iter() {
                let mut sensor_mut = sensor.borrow_mut();
                sensor_mut.cleanup_precomputed();
            }
        }
        self.normalize_weights().await;
    }

    pub async fn resample(&self) {
        let mut positions = self.positions.lock().await;
        let mut new_positions = self.new_positions.lock().await;
        let mut weights = self.weights.lock().await;
        let mut rng = self.generator.lock().await;

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
        let offset = dist.sample(&mut *rng);

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

    pub async fn calculate_mean(&self) {
        let positions = self.positions.lock().await;
        let weights = self.weights.lock().await;
        let mut estimate_position = self.estimate_position.lock().await;

        *estimate_position = positions.deref() * weights.deref().transpose();
    }

    pub async fn scatter_particles(&self, center: &Vector3<f32>, distance: f32) {
        let noise = {
            let mut rng = self.generator.lock().await;
            Matrix2xX::from_fn(self.particle_count, |_, _| {
                rng.random_range(-distance..distance)
            })
        };

        let mut positions = self.positions.lock().await;

        for i in 0..positions.ncols() {
            positions.set_column(i, center);
        }

        positions.fixed_view_mut::<2, 1>(0, 0).copy_from(&noise);
        core::mem::drop(positions);
        let mut weights = self.weights.lock().await;
        Matrix::fill(weights.deref_mut(), 1.0 / self.particle_count as f32);
    }

    async fn normalize_weights(&self) {
        let mut weights = self.weights.lock().await;
        let sum = weights.sum() + f32::MIN_POSITIVE * self.particle_count as f32;
        *weights = (weights.clone().map(|x| x + f32::MIN_POSITIVE)) / sum;
    }

    pub async fn run_filter(
        &self,
        delta_odometry: Vector3<f32>,
        sensors: Rc<Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>>,
    ) -> Option<nalgebra::Vector3<f32>> {
        if self.enabled {
            self.predict(&delta_odometry).await;
            self.update(sensors.clone()).await;
            self.resample().await;
            self.calculate_mean().await;
            Some(*self.estimate_position.lock().await)
        } else {
            None
        }
    }
}
