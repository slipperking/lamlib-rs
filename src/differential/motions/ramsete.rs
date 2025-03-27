use alloc::{boxed::Box, rc::Rc};
use core::{f64::consts::PI, time::Duration};

use bon::{bon, Builder};
use nalgebra::{Vector2, Vector3};
use num_traits::{AsPrimitive, Num};
use vexide::{float::Float, prelude::Motor};

use super::ToleranceGroup;
use crate::{
    controllers::FeedbackController,
    differential::{chassis::Chassis, pose::Pose},
    tracking::Tracking,
    utils::{
        math::{angle_error, arcade_desaturate, delta_clamp},
        timer::Timer,
    },
};

#[derive(Clone, Copy, PartialEq, Builder)]
pub struct RAMSETEHybridParameters {
    /// The `b` here, if set to `Some(_)`, overrides that of [`RAMSETEHybridSettings`].
    pub b: Option<f64>,

    #[builder(default = true)]
    pub forwards: bool,

    #[builder(default = 0.0)]
    pub min_linear_speed: f64,

    #[builder(default = 1.0)]
    pub max_linear_speed: f64,

    #[builder(default = 1.0)]
    pub max_angular_speed: f64,

    #[builder(default = 0.0)]
    pub early_exit_range: f64,
    pub linear_slew: Option<f64>,
    pub angular_slew: Option<f64>,
}

#[derive(Clone)]
/// Uses a hybrid between a RAMSETE controller and a closed loop feedback controller
/// such as PID.
pub struct RAMSETEHybridSettings {
    linear_controller: Box<dyn FeedbackController<f64>>,
    angular_controller: Box<dyn FeedbackController<f64>>,

    linear_tolerances: ToleranceGroup<f64>,
    angular_tolerances: ToleranceGroup<f64>,

    b: f64,
}

impl RAMSETEHybridSettings {
    pub fn new(
        linear_controller: Box<dyn FeedbackController<f64>>,
        angular_controller: Box<dyn FeedbackController<f64>>,
        linear_tolerances: ToleranceGroup<f64>,
        angular_tolerances: ToleranceGroup<f64>,
        b: f64,
    ) -> Self {
        Self {
            linear_controller,
            angular_controller,
            linear_tolerances,
            angular_tolerances,
            b,
        }
    }

    pub fn reset(&mut self) {
        self.linear_controller.reset();
        self.angular_controller.reset();
        self.linear_tolerances.reset();
        self.angular_tolerances.reset();
    }
}

#[macro_export]
macro_rules! params_ramsete_h {
    (
        $($key:ident : $value:expr),* $(,)?
    ) => {
        $crate::differential::motions::ramsete::RAMSETEHybridParameters::builder()
            $(.$key($value))*
            .build()
    };
}
pub use params_ramsete_h;

// Control scheme:
// First calculate local error: rotation matrix of (-current heading) * (target - current).
// Calculate v_d as PID(|e| * sign(cos(e_θ))).
// Then calculate angular_output as PID(e_θ) + b * v_d * e_y * sinc(e_θ).
// Calculate v as |cos(e_θ)| * v_d.

#[derive(Clone, PartialEq, Debug)]
pub enum RAMSETETarget {
    Pose(Pose),
    Point(Vector2<f64>),
}

impl RAMSETETarget {
    pub fn pose<T: Num + AsPrimitive<f64>, U: Num + AsPrimitive<f64>, V: Num + AsPrimitive<f64>>(
        x: T,
        y: U,
        orientation: V,
    ) -> Self {
        Self::Pose(Pose::new(x.as_(), y.as_(), orientation.as_()))
    }
    pub fn point<T: Num + AsPrimitive<f64>, U: Num + AsPrimitive<f64>>(x: T, y: U) -> Self {
        Self::Point(Vector2::new(x.as_(), y.as_()))
    }
}
impl<T: AsPrimitive<f64>, U: AsPrimitive<f64>> From<(T, U)> for RAMSETETarget {
    fn from((x, y): (T, U)) -> Self {
        Self::Point(Vector2::new(x.as_(), y.as_()))
    }
}

impl<T: AsPrimitive<f64>, U: AsPrimitive<f64>, V: AsPrimitive<f64>> From<(T, U, V)>
    for RAMSETETarget
{
    fn from((x, y, orientation): (T, U, V)) -> Self {
        Self::Pose(Pose::new(x.as_(), y.as_(), orientation.as_()))
    }
}

#[bon]
impl<T: Tracking + 'static> Chassis<T> {
    #[builder]
    pub async fn ramsete_hybrid(
        self: Rc<Self>,
        target: impl Into<RAMSETETarget> + 'static,
        timeout: Option<Duration>,
        params: Option<RAMSETEHybridParameters>,
        mut settings: Option<RAMSETEHybridSettings>,
        run_async: Option<bool>,
    ) {
        self.motion_handler.wait_for_motions_end().await;
        if self.motion_handler.is_in_motion() {
            return;
        }
        if run_async.unwrap_or(true) {
            // Spawn vexide task
            vexide::task::spawn({
                let self_clone = self.clone();
                async move {
                    self_clone
                        .ramsete_hybrid()
                        .target(target)
                        .maybe_timeout(timeout)
                        .maybe_params(params)
                        .maybe_settings(settings.clone())
                        .run_async(false)
                        .call()
                        .await
                }
            })
            .detach();
            self.motion_handler.end_motion().await;
            vexide::time::sleep(Duration::from_millis(10)).await;
            return;
        }
        if let Some(settings) = &mut settings {
            settings.reset();
        } else {
            self.motion_settings
                .ramsete_hybrid_settings
                .borrow_mut()
                .reset();
        }
        *self.distance_traveled.borrow_mut() = Some(0.0);
        let target: RAMSETETarget = target.into();
        let mut unwrapped_params = params.unwrap_or(params_ramsete_h!());
        unwrapped_params.min_linear_speed = unwrapped_params.min_linear_speed.abs();
        unwrapped_params.max_linear_speed = unwrapped_params.max_linear_speed.abs();
        unwrapped_params.max_angular_speed = unwrapped_params.max_angular_speed.abs();
        assert!(
            unwrapped_params.max_angular_speed < unwrapped_params.min_linear_speed,
            "Minimum speed may not exceed the maximum."
        );
        let mut previous_pose = self.pose().await;
        let mut is_near = false; // Possibly use settling logic.
        let mut previous_linear_output: f64 = 0.0;
        let mut previous_angular_output: f64 = 0.0;
        let mut timer = Timer::new(timeout.unwrap_or(Duration::MAX));
        while !timer.is_done() && self.motion_handler.is_in_motion() {
            let pose: Pose = self.pose().await;
            if let Some(distance_traveled) = self.distance_traveled.borrow_mut().as_mut() {
                *distance_traveled += pose.distance_to(&previous_pose);
            }
            previous_pose = pose;
            let mut local_error = Pose::from(
                // Similar to ∇×v, by the right hand rule, a negative direction
                // value is a clockwise rotation (negative orientation).
                nalgebra::Rotation3::new(Vector3::<f64>::new(0.0, 0.0, -pose.orientation))
                    * <Pose as Into<Vector3<f64>>>::into(
                        match target {
                            RAMSETETarget::Point(target_point) => {
                                Pose::new(target_point.x, target_point.y, {
                                    let error = target_point - pose.position;
                                    error.y.atan2(error.x)
                                })
                            }
                            RAMSETETarget::Pose(target_pose) => target_pose,
                        } - pose,
                    ),
            );
            if !is_near && local_error.position.norm() < 5.0 {
                is_near = true;
                unwrapped_params.max_linear_speed = previous_linear_output.max(0.5);
            }
            local_error.orientation = angle_error(
                local_error.orientation,
                if unwrapped_params.forwards { 0.0 } else { PI },
                true,
                None,
            );

            if if let Some(settings) = &mut settings {
                let linear_done = settings
                    .linear_tolerances
                    .update_all(local_error.position.norm());
                let angular_done = if settings
                    .angular_tolerances
                    .update_all(local_error.orientation)
                {
                    true
                } else {
                    matches!(target, RAMSETETarget::Point(_)) // Point-based RAMSETE does not check angular tolerances.
                };
                linear_done && angular_done
            } else {
                let mut motion_settings = self.motion_settings.boomerang_settings.borrow_mut();
                let linear_done = motion_settings
                    .linear_tolerances
                    .update_all(local_error.position.norm());
                let angular_done = if motion_settings
                    .angular_tolerances
                    .update_all(local_error.orientation)
                {
                    true
                } else {
                    matches!(target, RAMSETETarget::Point(_)) // Point-based RAMSETE does not check angular tolerances.
                };
                linear_done && angular_done
            } && is_near
            {
                break;
            }

            let v_d = {
                let controller_input =
                    local_error.position.norm() * local_error.orientation.cos().signum();
                if let Some(settings) = &mut settings {
                    settings.linear_controller.update(controller_input)
                } else {
                    self.motion_settings
                        .ramsete_hybrid_settings
                        .borrow_mut()
                        .linear_controller
                        .update(controller_input)
                }
            };
            let linear_output = (v_d * local_error.orientation.cos().abs()).clamp(
                -unwrapped_params.max_linear_speed,
                unwrapped_params.max_linear_speed,
            );
            let linear_output = {
                let check_1_result = if (-unwrapped_params.min_linear_speed
                    ..unwrapped_params.min_linear_speed)
                    .contains(&linear_output)
                {
                    unwrapped_params.min_linear_speed
                } else {
                    linear_output
                };
                let check_2_result = if check_1_result < 0.0 && !is_near {
                    0.0
                } else {
                    check_1_result
                };
                delta_clamp(
                    check_2_result,
                    previous_linear_output,
                    unwrapped_params.linear_slew.unwrap_or(0.0),
                    None,
                )
            };
            let angular_output = ({
                if let Some(settings) = &mut settings {
                    settings.angular_controller.update(local_error.orientation)
                } else {
                    self.motion_settings
                        .ramsete_hybrid_settings
                        .borrow_mut()
                        .angular_controller
                        .update(local_error.orientation)
                }
            } + v_d
                * local_error.position.y
                * {
                    use nalgebra::ComplexField;
                    local_error.orientation.sinc()
                }
                * {
                    unwrapped_params
                        .b
                        .unwrap_or(if let Some(settings) = &settings {
                            settings.b
                        } else {
                            self.motion_settings.ramsete_hybrid_settings.borrow().b
                        })
                })
            .clamp(
                -unwrapped_params.max_angular_speed,
                unwrapped_params.max_angular_speed,
            );
            let angular_output = delta_clamp(
                angular_output,
                previous_angular_output,
                unwrapped_params.angular_slew.unwrap_or(0.0),
                None,
            );
            previous_linear_output = linear_output;
            previous_angular_output = angular_output;

            let (left, right) = arcade_desaturate(
                if unwrapped_params.forwards {
                    linear_output
                } else {
                    -linear_output
                },
                angular_output,
            );

            self.drivetrain
                .left_motors
                .borrow_mut()
                .set_velocity_percentage_all(left);
            self.drivetrain
                .right_motors
                .borrow_mut()
                .set_velocity_percentage_all(right);

            vexide::time::sleep(Motor::WRITE_INTERVAL).await;
        }
        *self.distance_traveled.borrow_mut() = None;
        self.motion_handler.end_motion().await;
    }
}
