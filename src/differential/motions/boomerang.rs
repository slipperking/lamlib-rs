use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use nalgebra::Vector3;

use crate::{differential::chassis::Chassis, tracking::Tracking};

#[derive(Clone, Copy, PartialEq)]
pub struct BoomerangParameters {
    pub forwards: bool,
    pub min_speed: f64,
    pub max_speed: f64,
    pub early_exit_range: f64,
}
#[macro_export]
macro_rules! boomerang {
    (
        $(forwards => $forwards:expr;)?
        $(min_speed => $min_speed:expr;)?
        $(max_speed => $max_speed:expr;)?
        $(early_exit_range => $early_exit_range:expr;)?
    ) => {{
        let forwards = true;
        let min_speed = 0.0;
        let max_speed = 12.0;
        let early_exit_range = 0.0;

        $(forwards = $forwards;)?
        $(min_speed = $min_speed;)?
        $(max_speed = $max_speed;)?
        $(early_exit_range = $early_exit_range;)?

        BoomerangParameters {
            forwards, min_speed, max_speed, early_exit_range
        }}
    }
}
pub use boomerang;

impl<T: Tracking + 'static> Chassis<T> {
    async fn boomerang(
        self: Rc<Self>,
        boomerang_target: Vector3<f64>,
        timeout: Option<Duration>,
        params: Option<BoomerangParameters>,
        run_async: bool,
    ) {
        let unwrapped_params = params.unwrap_or(boomerang!());
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
        let mut motion_settings = self.motion_settings.borrow_mut();
        motion_settings.reset();
        drop(motion_settings);
        todo!();
    }
}
