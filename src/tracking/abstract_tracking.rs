use alloc::{boxed::Box, rc::Rc};

use nalgebra::Vector3;
use vexide::core::sync::Mutex;

#[async_trait::async_trait(?Send)]
pub trait Tracking {
    fn position(&mut self) -> Vector3<f64>;
    async fn set_position(&mut self, position: &Vector3<f64>);

    /// A Reference Counted Pointer with a Mutex is required for the pointer.
    /// NOTE: In implementation, you are not required to use self_rc.
    /// Do not synchronously attempt to lock the mutex in your code.
    async fn init(&mut self, async_self_rc: Rc<Mutex<Self>>);
}
