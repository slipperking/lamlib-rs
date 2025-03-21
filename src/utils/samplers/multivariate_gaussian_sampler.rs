use nalgebra::{ArrayStorage, Const, Dyn, Matrix, SMatrix, SVector, VecStorage};
use rand::Rng;

use super::Sampler;

/// Gaussian sampler that can sample a batch or a single value.
/// Takes in a covariance matrix and the mean vector.
/// T: Vector dimension,
/// P: Precompute size,
/// To improve efficiency, a large sample size of precomputed values will be made and randomly drawn from.
pub struct GaussianSampler<const T: usize, const P: usize> {
    precomputed_size: usize,
    precomputed_samples: SMatrix<f32, T, P>,
    rng: veranda::SystemRng,
}

impl<const T: usize, const P: usize> GaussianSampler<T, P> {
    pub fn new(
        mu: SVector<f32, T>,
        sigma: Matrix<f32, Const<T>, Const<T>, ArrayStorage<f32, T, T>>,
    ) -> Self {
        let mut rng = veranda::SystemRng::new();
        let mut precomputed_samples = SMatrix::<f32, T, P>::zeros();

        // Cholesky decomposition for positive definite covariance.
        let l = sigma
            .cholesky()
            .expect("Matrix must be positive definite")
            .l();

        // Precompute a set of samples.
        for j in 0..P {
            let standard_normal = SVector::<f32, T>::from_fn(|_, _| rng.random::<f32>());
            precomputed_samples.set_column(j, &(mu + l * standard_normal));
        }

        Self {
            precomputed_size: P,
            precomputed_samples,
            rng,
        }
    }
}

impl<const T: usize, const P: usize> Sampler<T> for GaussianSampler<T, P> {
    fn sample(&mut self) -> SVector<f32, T> {
        let idx = self.rng.random_range(0..self.precomputed_size);
        self.precomputed_samples.column(idx).into()
    }

    fn sample_batch(
        &mut self,
        n: usize,
    ) -> Matrix<f32, Const<T>, Dyn, VecStorage<f32, Const<T>, Dyn>> {
        let mut samples = Matrix::<f32, Const<T>, Dyn, VecStorage<f32, Const<T>, Dyn>>::zeros(n);
        for j in 0..n {
            let idx = self.rng.random_range(0..self.precomputed_size);
            samples.set_column(j, &self.precomputed_samples.column(idx));
        }
        samples
    }
}
