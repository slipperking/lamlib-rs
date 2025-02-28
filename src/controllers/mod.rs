use num::FromPrimitive;
use num_traits::{Float, Num, NumAssign, NumAssignOps, NumAssignRef};
pub trait ControllerMethod<T: Float + Num + NumAssign + NumAssignOps + NumAssignRef + FromPrimitive>
{
    fn update(&mut self, error: T) -> T;
    fn reset(&mut self);
}

#[macro_use]
pub mod pid;
