use alloc::boxed::Box;

use async_trait::async_trait;
use vexide::io::println;

use super::AutonRoutine;
use crate::{differential::pose::Pose, params_boomerang, utils::math::AngleExt, Robot};
pub struct Skills;
#[async_trait(?Send)]
impl AutonRoutine for Skills {
    fn name() -> &'static str {
        "Skills"
    }

    fn color() -> crate::utils::AllianceColor {
        crate::utils::AllianceColor::Red
    }

    fn symbol() -> &'static str {
        "skl "
    }

    async fn run(&self, robot: &mut Robot) {
        robot
            .chassis
            .clone()
            .set_pose(Pose::new(-60.0, 0.0, -90.0.hdg_deg()))
            .await;
        robot
            .chassis
            .clone()
            .boomerang()
            .target(Pose::new(-50.0, -10.0, 0.0.hdg_deg()))
            .params(params_boomerang!(max_lateral_speed: 0.8,))
            .call()
            .await;
        robot.chassis.clone().wait_until_complete().await;
        println!("{}", Skills::color().get_name());
    }
}
