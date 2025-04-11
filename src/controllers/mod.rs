use num_traits::Float;

use crate::utils::differential_tracker::DifferentialTracker;

dyn_clone::clone_trait_object!(<T> FeedbackController<T>);
pub trait FeedbackController<T: Float>: dyn_clone::DynClone {
    fn update(&mut self, set_point: T, process_variable: T) -> T;
    fn reset(&mut self);
}

dyn_clone::clone_trait_object!(<T> FeedforwardController<T>);
pub trait FeedforwardController<T: Float>: dyn_clone::DynClone {
    fn update(&mut self, differential_tracker: DifferentialTracker<T>) -> T;
    fn reset(&mut self);
}

pub mod pid;
