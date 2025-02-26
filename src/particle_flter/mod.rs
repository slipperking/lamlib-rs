pub mod sensors;
use alloc::{rc::Rc, vec::Vec};

use nalgebra::{Const, Dyn, VecStorage};
use sensors::ParticleFilterSensor;
use vexide::core::sync::Mutex;

use crate::utils::samplers::*;

pub struct Localization<const T: usize, P: ParticleFilterSensor> {
    positions: Mutex<nalgebra::Matrix<f32, Const<3>, Dyn, VecStorage<f32, Const<3>, Dyn>>>,
    weights: Mutex<nalgebra::DVector<f32>>,
    sampler: multivariate_gaussian_sampler::GaussianSampler<3, 10000>,
    sensors: Vec<Rc<P>>,
    enabled: bool,
}
