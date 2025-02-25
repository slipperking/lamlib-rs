use alloc::rc::Rc;

use nalgebra::Vector3;
use vexide::core::sync::Mutex;
pub trait Tracking {
    fn position(&mut self) -> Vector3<f64>;

    /// A Reference Counted Pointer with a Mutex is required for the pointer.
    async fn init(&mut self, self_rc: Rc<Mutex<Self>>);
}
