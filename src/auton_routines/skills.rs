use alloc::{boxed::Box, rc::Rc};
use core::time::Duration;

use async_trait::async_trait;
use nalgebra::Vector2;
use vexide::io::println;

use super::AutonRoutine;
use crate::{
    differential::{
        motions::angular::{TurnToParameters, TurnToTarget},
        pose::Pose,
    },
    utils::math::AngleExt,
    Robot,
};
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
        let chassis = robot.chassis.clone();
        let intake = robot.intake.clone();
        Rc::clone(&chassis)
            .set_pose(Pose::new(-62.0, 0.0, -90.0.hdg_deg()))
            .await;

        {
            intake.lock().await.spin();
        }
        vexide::time::sleep(Duration::from_millis(500)).await;
        {
            intake.lock().await.stop();
        }

        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-52.0, -10.0))
            .params(params_move_to_point!(forwards: false, min_linear_speed: 0.2, early_exit_range: 2.0,))
            .call()
            .await;

        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::Point(Vector2::new(-49.0, -24.0)))
            .run_async(false)
            .call()
            .await;
        robot.clamp_main.set_state(true);
        {
            intake.lock().await.spin();
        }
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::Point(Vector2::new(-26.0, -23.0)))
            .params(TurnToParameters::builder().forwards(false).build())
            .run_async(false)
            .call()
            .await;
        println!("{}", Skills::color().get_name());
    }
}
