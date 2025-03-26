pub mod exponential;
pub mod flipped_power;
pub mod power;

dyn_clone::clone_trait_object!(DriveCurve);

pub trait DriveCurve: dyn_clone::DynClone {
    fn update(&self, input: f64) -> f64;
}
