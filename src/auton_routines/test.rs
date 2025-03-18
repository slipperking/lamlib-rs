use alloc::boxed::Box;

use async_trait::async_trait;
use nalgebra::Vector2;
use vexide::io::println;

use super::AutonRoutine;
use crate::{
    differential::{motions::ramsete::RamseteTarget, pose::Pose},
    Robot,
};
pub struct Test;

#[async_trait(?Send)]
impl AutonRoutine for Test {
    fn name() -> &'static str {
        "Test"
    }

    fn color() -> crate::utils::AllianceColor {
        crate::utils::AllianceColor::Red
    }

    fn symbol() -> &'static str {
        "test"
    }

    async fn run(&self, robot: &mut Robot) {
        robot
            .chassis
            .set_pose(Pose::new(0.0, 0.0, angle!(degrees: 0.0, standard: true,))).await;
        robot.chassis.clone().ramsete_hybrid(
            RamseteTarget::Point(Vector2::new(24.0, 0.0)),
            None,
            None,
            None,
            true,
        ).await;
        println!("{}", Test::color().get_name());
    }
}
