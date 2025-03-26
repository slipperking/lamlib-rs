pub mod odom;

use alloc::{boxed::Box, rc::Rc};

use nalgebra::Vector3;
use vexide::sync::Mutex;

#[async_trait::async_trait(?Send)]
pub trait Tracking {
    fn position(&mut self) -> Vector3<f64>;
    async fn set_position(&mut self, position: &Vector3<f64>);
    async fn set_filter_state(&mut self, state: bool);
    async fn filter_state(&self) -> Option<bool>;

    /// A Reference Counted Pointer with a Mutex is required for the pointer.
    /// NOTE: In implementation, you are not required to use self_rc.
    /// Do not synchronously attempt to lock the mutex in your code.
    async fn init(&mut self, self_rc_mutex: Rc<Mutex<Self>>);
}
