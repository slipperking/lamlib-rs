use alloc::{boxed::Box, string::ToString};
use core::time::Duration;

use async_trait::async_trait;
use vexide::io::{println, Write};

use super::AutonRoutine;
use crate::{
    params_turn_to,
    utils::{math::AngleExt, timer::Timer, TILE_SIZE},
    Robot,
};
pub struct Test;
pub enum TestMode {
    Linear(f64),
    Angular(f64),
    TrackingCenter(Duration, f64),
    ImuScalar,
    ParticleFilter,
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
        chassis.set_pose((0.0, 0.0, 0.0.hdg_deg())).await;
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
                println!("{}", robot.chassis.pose().await.to_string());
            }
            TestMode::Angular(angle) => {
                chassis
                    .clone()
                    .turn_to()
                    .target((
                        robot.chassis.pose().await.orientation + angle,
                    ))
                    .params(params_turn_to!(forwards: false,))
                    .run_async(false)
                    .call()
                    .await;

                println!(
                    "{}",
                    unsigned_mod!(robot.chassis.pose().await.orientation.to_degrees(), 360.0)
                        as f32
                );
            }
            TestMode::TrackingCenter(duration, velocity_percentage) => {
                println!("\x1b[1mCopy this:\x1b[0m\n\\left[");
                let mut timer = Timer::new(duration);
                let mut movement_index = 0;

                while !timer.is_done() {
                    chassis.arcade(0.0, velocity_percentage, false);
                    let pose = chassis.pose().await;

                    vexide::io::print!("\\left({},{}\\right),", pose.position.x, pose.position.y);

                    // Flush every 50 iterations.
                    movement_index += 1;
                    if movement_index % 50 == 0 {
                        vexide::io::stdout().lock().await.flush().unwrap();
                    }

                    vexide::time::sleep(Duration::from_millis(20)).await;
                }
                println!("\x08\\right]");
                println!(
                    "Go to https://www.desmos.com/calculator/rxdoxxil1j to solve for offsets."
                );

                chassis.arcade(0.0, 0.0, false);
            }
            TestMode::ImuScalar => {}
            TestMode::ParticleFilter => {
                if let Some(filter_state) = chassis.filter_state().await {
                    chassis.set_filter_state(!filter_state).await;
                }
            }
            TestMode::Default => {
                chassis
                    .clone()
                    .ramsete_hybrid()
                    .target((TILE_SIZE, TILE_SIZE, 0.0.hdg_deg()))
                    .run_async(false)
                    .call()
                    .await;

                chassis
                    .clone()
                    .ramsete_hybrid()
                    .target((-TILE_SIZE, TILE_SIZE, 180.0.hdg_deg()))
                    .params(params_ramsete_h!(forwards: false,))
                    .call()
                    .await;
            }
        }
        println!("{}", Test::color().get_name());
    }
}
