use alloc::boxed::Box;
use core::time::Duration;

use async_trait::async_trait;
use vexide::io::println;

use super::AutonRoutine;
use crate::{
    differential::{
        motions::{angular::TurnToTarget, ramsete::RAMSETETarget},
        pose::Pose,
    },
    params_ramsete_h, params_turn_to,
    utils::{math::AngleExt, TILE_SIZE},
    Robot,
};
pub struct Test;
pub enum TestMode {
    Linear(f64),
    Angular(f64),
    TrackingCenter(Duration, f64),
    ImuScalar,
    Default,
}

static TEST_MODE: TestMode = TestMode::Linear(TILE_SIZE);

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
        let chassis = robot.chassis.clone();
        chassis.set_pose(Pose::new(0.0, 0.0, 0.0.hdg_deg())).await;
        match TEST_MODE {
            TestMode::Linear(distance) => {
                // Also tests concurrent actions.
                chassis
                    .clone()
                    .move_relative()
                    .distance(distance)
                    .run_async(false)
                    .call()
                    .await;
                robot.intake.lock().await.spin();
                chassis.wait_until_complete().await;

                robot.intake.lock().await.stop();
                println!("{:?}", robot.chassis.pose().await);
            }
            TestMode::Angular(angle) => {
                chassis
                    .clone()
                    .turn_to()
                    .target(TurnToTarget::Angle(
                        robot.chassis.pose().await.orientation + angle,
                    ))
                    .params(params_turn_to!(forwards: false,))
                    .run_async(false)
                    .call()
                    .await;

                println!("{}", robot.chassis.pose().await.orientation);
            }
            TestMode::TrackingCenter(duration, velocity_percentage) => {
                chassis.set_pose(Pose::new(0.0, 0.0, 0.0.std_deg())).await;
                _ = duration;
                _ = velocity_percentage;
            }
            TestMode::ImuScalar => {}
            TestMode::Default => {
                chassis
                    .clone()
                    .ramsete_hybrid()
                    .target(RAMSETETarget::pose(TILE_SIZE, TILE_SIZE, 0.0.hdg_deg()))
                    .run_async(false)
                    .call()
                    .await;

                chassis
                    .clone()
                    .ramsete_hybrid()
                    .target(RAMSETETarget::pose(-TILE_SIZE, TILE_SIZE, 180.0.hdg_deg()))
                    .params(params_ramsete_h!(forwards: false,))
                    .call()
                    .await;
            }
        }
        println!("{}", Test::color().get_name());
    }
}
