use core::ops::AddAssign;

use num::FromPrimitive;
use num_traits::Float;
pub trait ControllerMethod<T: Float + FromPrimitive + AddAssign>: dyn_clone::DynClone {
    fn update(&mut self, error: T) -> T;
    fn reset(&mut self);
}

#[macro_use]
pub mod pid;
