use alloc::rc::Rc;
use core::{f64::consts::PI, time::Duration};

use vexide::prelude::Float;

use crate::{
    differential::{chassis::Chassis, pose::Pose},
    tracking::Tracking,
    utils::math::{angle_error, AngularDirection},
};

#[derive(Clone, Copy, PartialEq)]
pub struct BoomerangParameters {
    pub forwards: bool,
    pub lead: f64,
    pub min_lateral_speed: f64,
    pub max_lateral_speed: f64,
    pub early_exit_range: f64,
}

#[macro_export]
macro_rules! boomerang {
    (
        $(forwards => $forwards:expr;)?
        $(lead => $lead:expr;)?
        $(min_lateral_speed => $min_lateral_speed:expr;)?
        $(max_lateral_speed => $max_lateral_speed:expr;)?
        $(early_exit_range => $early_exit_range:expr;)?
    ) => {
        {
            let forwards = true;
            let lead = 0.6;
            let min_lateral_speed = 0.0;
            let max_lateral_speed = 12.0;
            let early_exit_range = 0.0;

            $(forwards = $forwards;)?
            $(lead = $lead;)?
            $(min_lateral_speed = $min_lateral_speed;)?
            $(max_lateral_speed = $max_lateral_speed;)?
            $(early_exit_range = $early_exit_range;)?

            BoomerangParameters {
                forwards, lead, min_lateral_speed, max_lateral_speed, early_exit_range
            }
        }
    }
}
pub use boomerang;

impl<T: Tracking + 'static> Chassis<T> {
    /// Consider boomerang to be lateral; distance traveled, etc. will be distance in inches.
    pub async fn boomerang(
        self: Rc<Self>,
        mut boomerang_target: Pose,
        timeout: Option<Duration>,
        params: Option<BoomerangParameters>,
        run_async: bool,
    ) {
        let mut unwrapped_params = params.unwrap_or(boomerang!());
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
            motion_settings.reset();
        }
        let mut previous_pose = self.pose().await;
        {
            *self.distance_traveled.borrow_mut() = Some(0.0);
        }
        let mut is_near = false;
        let mut is_lateral_settled = false;
        let mut previous_was_same_side = false;
        let mut previous_lateral_output: f64 = 0.0;
        let mut previous_angular_output: f64 = 0.0;
        loop {
            let pose = self.pose().await;
            {
                if let Some(distance) = self.distance_traveled.borrow_mut().as_mut() {
                    *distance += pose.distance_to(&previous_pose);
                }
            }
            let distance_to_target = pose.distance_to(&boomerang_target);
            previous_pose = pose;
            if distance_to_target < 7.5 && !is_near {
                is_near = true;
                unwrapped_params.max_lateral_speed = previous_lateral_output.max(6.0);
            }
            let robot_vectors_normalized = nalgebra::Vector2::<f64>::new(
                boomerang_target.orientation.cos(),
                boomerang_target.orientation.sin(),
            );
            let carrot_point = if is_near {
                boomerang_target.position
            } else {
                boomerang_target.position
                    - robot_vectors_normalized * unwrapped_params.lead * distance_to_target
            };
            let carrot_pose_angle =
                (carrot_point.y - pose.position.y).atan2(carrot_point.x - pose.position.x);
            let lateral_error = {
                // Find the cosine of the signed angle between.
                let cos_angle_difference = angle_error(
                    pose.orientation,
                    carrot_pose_angle,
                    true,
                    AngularDirection::Auto,
                )
                .cos();
                if is_near {
                    distance_to_target * cos_angle_difference
                } else {
                    distance_to_target * cos_angle_difference.signum()
                }
            };
            let angular_error = {
                let adjusted_orientation = if unwrapped_params.forwards {
                    pose.orientation + PI
                } else {
                    pose.orientation
                };
                if is_near {
                    angle_error(
                        adjusted_orientation,
                        boomerang_target.orientation,
                        true,
                        AngularDirection::Auto,
                    )
                } else {
                    angle_error(
                        adjusted_orientation,
                        carrot_pose_angle,
                        true,
                        AngularDirection::Auto,
                    )
                }
            };
            {
                let mut motion_settings = self.motion_settings.borrow_mut();
                if ('lateral_checks: {
                    for lateral_exit_condition in &mut motion_settings.lateral_exit_conditions {
                        if lateral_exit_condition.update(lateral_error) {
                            break 'lateral_checks true;
                        }
                    }
                    false
                } && 'angular_checks: {
                    for angular_exit_condition in &mut motion_settings.angular_exit_conditions {
                        if angular_exit_condition.update(angular_error) {
                            break 'angular_checks true;
                        }
                    }
                    false
                } && is_near)
                {
                    break;
                }
            }
            vexide::time::sleep(vexide::prelude::Motor::WRITE_INTERVAL).await;
        }
        todo!();
    }
}
