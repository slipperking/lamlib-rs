use nalgebra::{ArrayStorage, Const, Dyn, Matrix, SMatrix, SVector, VecStorage};
use rand::Rng;

use super::BatchSampler;

/// Gaussian sampler that can sample a batch or a single value, taking in the
/// covariance matrix and a mean vector.
///
/// The sampler precomputes a set of samples to draw from, which can be used to
/// reduce time required to generate samples when they are sampled from. Provide a high value
/// for `P` to increase the sample size and increase randomness.
///
/// # Arguments:
/// - `T`: The vector dimension (column size).
/// - `P`: Precompute size, or the number of samples to precompute.
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

impl<const T: usize, const P: usize> BatchSampler<T> for GaussianSampler<T, P> {
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
