use alloc::rc::Rc;
use core::future::Future;

use nalgebra::Vector3;
use vexide::core::sync::Mutex;
pub trait Tracking {
    fn position(&mut self) -> Vector3<f64>;
    fn set_position(&mut self, position: &Vector3<f64>);

    /// A Reference Counted Pointer with a Mutex is required for the pointer.
    /// NOTE: In implementation, you are not required to use self_rc.
    /// Do not synchronously attempt to lock the mutex in your code.
    fn init(&mut self, async_self_rc: Rc<Mutex<Self>>) -> impl Future<Output = ()>;
}
