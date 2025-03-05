use super::super::chassis::Chassis;
use crate::tracking::abstract_tracking::Tracking;

pub struct RamseteParams {
    forwards: bool,
    min_speed: f64,
    max_speed: f64,
    early_exit_range: f64,
}

impl<T: Tracking> Chassis<T> {
    pub async fn ramsete(target: &nalgebra::Vector2<f64>) {}
}
