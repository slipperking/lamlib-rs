use nalgebra::{Const, Dyn, Matrix, RowDVector, VecStorage};

pub mod distance;

// #[async_trait::async_trait(?Send)]
pub trait ParticleFilterSensor<const T: usize> {
    fn precompute(
        &mut self,
        positions: &Matrix<f32, Const<T>, Dyn, VecStorage<f32, Const<T>, Dyn>>,
    );
    fn cleanup_precomputed(&mut self);
    fn update(
        &mut self,
        positions: &Matrix<f32, Const<T>, Dyn, VecStorage<f32, Const<T>, Dyn>>,
        weights: &mut RowDVector<f32>,
    );
}
