use alloc::rc::Rc;
use core::time::Duration;

use nalgebra::Vector2;

use crate::{differential::chassis::Chassis, tracking::Tracking};

#[derive(Clone, Copy, PartialEq)]
pub struct MoveToPointParameters {}

#[derive(Clone)]
pub struct MoveToPointSettings {}

impl<T: Tracking + 'static> Chassis<T> {
    pub async fn move_to_point(
        self: Rc<Self>,
        target: Vector2<f64>,
        timeout: Option<Duration>,
        params: Option<MoveToPointParameters>,
        mut settings: Option<MoveToPointSettings>,
        run_async: bool,
    ) {
        self.motion_handler.wait_for_motions_end().await;
        if self.motion_handler.in_motion() {
            return;
        }
        if run_async {
            // Spawn vexide task
            vexide::task::spawn({
                let self_clone = self.clone();
                async move {
                    self_clone
                        .move_to_point(target, timeout, params, settings.clone(), false)
                        .await
                }
            })
            .detach();
            self.motion_handler.end_motion().await;
            vexide::time::sleep(Duration::from_millis(10)).await;
            return;
        }
    }
    pub async fn move_relative(self: Rc<Self>) {}
}
