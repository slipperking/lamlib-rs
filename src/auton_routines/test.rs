use alloc::boxed::Box;
use core::f64::consts::{FRAC_PI_2, PI};

use async_trait::async_trait;
use nalgebra::Vector2;
use vexide::{io::println, prelude::Float};

use super::AutonRoutine;
use crate::{
    differential::{
        motions::{angular::TurnTarget, ramsete::RamseteTarget},
        pose::Pose,
    },
    params_turn_to,
    utils::TILE_LENGTH,
    Robot,
};
pub struct Test;
enum TestMode {
    Lateral(f64),
    Angular(f64),
    TrackingCenter,
    ImuScalar,
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
            .set_pose(Pose::new(0.0, 0.0, angle!(degrees: 0.0, standard: false,)))
            .await;
        match TEST_MODE {
            TestMode::Lateral(distance) => {
                robot.chassis.clone();
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
            }
            TestMode::TrackingCenter => {}
            TestMode::ImuScalar => {}
        }
        println!("{}", Test::color().get_name());
    }
}
