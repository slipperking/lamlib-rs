use alloc::boxed::Box;

use async_trait::async_trait;
use vexide::io::println;

use super::AutonRoutine;
use crate::{
    differential::{
        motions::{angular::TurnTarget, ramsete::RamseteTarget},
        pose::Pose,
    },
    params_ramsete_h, params_turn_to,
    utils::TILE_LENGTH,
    Robot,
};
pub struct Test;
pub enum TestMode {
    Lateral(f64),
    Angular(f64),
    TrackingCenter,
    ImuScalar,
    Default,
}

static TEST_MODE: TestMode = TestMode::Lateral(TILE_LENGTH);

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
            .set_pose(Pose::new(0.0, 0.0, angle!(degrees: 0.0,)))
            .await;
        match TEST_MODE {
            TestMode::Lateral(distance) => {
                // Also tests concurrent actions.
                robot
                    .chassis
                    .clone()
                    .move_relative(distance, None, None, None, false)
                    .await;
                robot.intake.lock().await.spin();
                robot.chassis.wait_until_complete().await;

                robot.intake.lock().await.stop();
                println!("{:?}", robot.chassis.pose().await);
            }
            TestMode::Angular(angle) => {
                robot
                    .chassis
                    .clone()
                    .turn_to(
                        TurnTarget::Angle(robot.chassis.pose().await.orientation + angle),
                        None,
                        Some(params_turn_to!(forwards: false,)),
                        None,
                        false,
                    )
                    .await;

                println!("{}", robot.chassis.pose().await.orientation);
            }
            TestMode::TrackingCenter => {}
            TestMode::ImuScalar => {}
            TestMode::Default => {
                robot
                    .chassis
                    .clone()
                    .ramsete_hybrid(
                        RamseteTarget::Pose(Pose::new(
                            TILE_LENGTH,
                            TILE_LENGTH,
                            angle!(degrees: 0.0,),
                        )),
                        None,
                        None,
                        None,
                        false,
                    )
                    .await;

                robot
                    .chassis
                    .clone()
                    .ramsete_hybrid(
                        RamseteTarget::Pose(Pose::new(
                            -TILE_LENGTH,
                            TILE_LENGTH,
                            angle!(degrees: 180.0,),
                        )),
                        None,
                        Some(params_ramsete_h!(forwards: false,)),
                        None,
                        true,
                    )
                    .await;
            }
        }
        println!("{}", Test::color().get_name());
    }
}
