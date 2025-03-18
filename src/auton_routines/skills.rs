use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use vexide::io::println;

use super::AutonRoutine;
use crate::{differential::pose::Pose, params_boomerang, Robot};
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
            .set_pose(Pose::new(
                -60.0,
                0.0,
                angle!(degrees: -90.0, standard: false,),
            ))
            .await;
        robot
            .chassis
            .clone()
            .boomerang(
                Pose::new(-50.0, -10.0, angle!(degrees: 0.0, standard: false,)),
                Some(Duration::from_millis(3000)),
                Some(params_boomerang!(max_lateral_speed: 0.8,)),
                None,
                true,
            )
            .await;
        robot.chassis.clone().wait_until_complete().await;
        println!("{}", Skills::color().get_name());
    }
}
