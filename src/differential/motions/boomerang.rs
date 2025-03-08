use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use nalgebra::Vector3;

use crate::{differential::chassis::Chassis, tracking::Tracking};

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BoomerangParameters {
    pub forward: bool,
}

impl<T: Tracking + 'static> Chassis<T> {
    async fn boomerang(
        self: Rc<Self>,
        boomerang_target: Vector3<f64>,
        timeout: Option<Duration>,
        params: BoomerangParameters,
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
                        .boomerang(boomerang_target, timeout, params, false)
                        .await
                }
            })
            .detach();
            self.motion_handler.end_motion().await;
            vexide::time::sleep(Duration::from_millis(10)).await;
            return;
        }
        {
            let mut motion_settings = self.motion_settings.borrow_mut();
            motion_settings.lateral_pid.reset();
            motion_settings.lateral_pid.reset();
        }
        todo!();
    }
}
