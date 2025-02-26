use core::future::Future;

pub mod distance;
pub trait ParticleFilterSensor {
     fn update() -> impl Future<Output = ()>;
}