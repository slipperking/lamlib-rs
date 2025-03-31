use nalgebra::{Const, Dyn, Matrix, VecStorage};
pub mod multivariate_gaussian_sampler;

pub trait MultivariateSampler<const T: usize> {
    fn sample(&mut self) -> nalgebra::SVector<f32, T>;
    fn sample_batch(
        &mut self,
        n: usize,
    ) -> Matrix<f32, Const<T>, Dyn, VecStorage<f32, Const<T>, Dyn>> {
        Matrix::<f32, Const<T>, Dyn, VecStorage<f32, Const<T>, Dyn>>::from_columns(
            &(0..n)
                .map(|_| self.sample())
                .collect::<alloc::vec::Vec<_>>(),
        )
    }
}
