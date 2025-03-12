use nalgebra::Vector2;

use crate::utils::math::AngularDirection;

pub enum DriveSide {
    Left,
    Right,
}

pub struct TurnToParameters {
    pub forwards: bool,
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

    /// A heading the robot can turn to face.
    /// This is in standard radians.
    StandardRadians(f64),

    /// A heading the robot can turn to face.
    /// This is in standard degrees.
    StandardDegrees(f64),

    /// A heading the robot can turn to face.
    /// This is in heading radians.
    HeadingRadians(f64),

    /// A heading the robot can turn to face.
    /// This is in heading degrees.
    HeadingDegrees(f64),
}

