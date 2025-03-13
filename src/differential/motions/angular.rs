use nalgebra::Vector2;
use vexide::prelude::Float;

use crate::{differential::pose::Pose, utils::math::AngularDirection};

pub enum DriveSide {
    Left,
    Right,
}

pub struct TurnToParameters {
    pub forwards: bool,

    /// Speed constraints here restrict both drive sides.
    pub min_speed: f64,
    pub max_speed: f64,
    pub early_exit_range: f64,
    pub angular_slew: Option<f64>,
    pub direction: Option<AngularDirection>,

    /// The locked side for a swing turn.
    pub locked_side: Option<DriveSide>,
}

pub enum TurnTarget {
    /// A point the robot can turn to face.
    /// This is in inches.
    Point(Vector2<f64>),

    /// An angle the robot can turn to.
    ///
    /// # Example
    /// ```
    /// TurnTarget::Angle(angle!(degrees: 90.0, standard: None,))
    /// ```
    Angle(f64),
}

impl TurnTarget {
    /// The angle error between a pose and a target.
    ///
    pub fn error(&self, pose: Pose, direction: Option<AngularDirection>) -> f64 {
        match *self {
            Self::Point(point) => {
                let local_delta =
                    nalgebra::Rotation2::new(-pose.orientation) * (point - pose.position);
                f64::atan2(local_delta.y, local_delta.x)
            }
            Self::Angle(angle) => {
                crate::utils::math::angle_error(angle, pose.orientation, true, direction)
            }
        }
    }
}
