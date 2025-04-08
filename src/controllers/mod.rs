use core::ops::AddAssign;

use num_traits::FromPrimitive;
use vexide::float::Float;

dyn_clone::clone_trait_object!(<T> FeedbackController<T>);
pub trait FeedbackController<T: Float + FromPrimitive + AddAssign>: dyn_clone::DynClone {
    fn update(&mut self, error: T) -> T;
    fn reset(&mut self);
}

pub mod pid;
pub mod simple_feedforward;