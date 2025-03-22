use alloc::{boxed::Box, rc::Rc};
use core::time::Duration;

use async_trait::async_trait;
use nalgebra::Vector2;
use vexide::io::println;

use super::AutonRoutine;
use crate::{
    differential::{
        motions::{
            angular::{TurnToParameters, TurnToTarget},
            linear::{MoveRelativeParameters, MoveToPointParameters},
            ramsete::RAMSETETarget,
        },
        pose::Pose,
    },
    subsystems::ladybrown::LadyBrownState,
    utils::{math::AngleExt, AllianceColor},
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
        let ladybrown_arm = robot.ladybrown_arm.clone();
        chassis.set_filter_state(true).await;
        Rc::clone(&chassis)
            .set_pose(Pose::new(-62.0, 0.0, -90.0.hdg_deg()))
            .await;

        intake.lock().await.spin();
        vexide::time::sleep(Duration::from_millis(500)).await;
        intake.lock().await.stop();

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

        intake.lock().await.spin();

        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::Point(Vector2::new(-26.0, -23.0)))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .early_exit_range(2.0.deg())
                    .min_speed(0.2)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-28.0, -23.0))
            .params(
                MoveToPointParameters::builder()
                    .forwards(false)
                    .early_exit_range(2.0)
                    .min_linear_speed(0.1)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::Angle(135.0.hdg_deg()))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .early_exit_range(2.0.deg())
                    .min_speed(0.3)
                    .build(),
            )
            .call()
            .await;

        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(23.0, -47.0))
            .params(params_move_to_point!(forwards: false, min_linear_speed: 0.2, early_exit_range: 2.0,))
            .call()
            .await;

        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(46.0, -47.0))
            .params(params_move_to_point!(forwards: false, max_linear_speed: 0.7))
            .call()
            .await;
        chassis.wait_until(10.0).await;

        intake.lock().await.set_optical_callback(Box::new({
            let lady_brown = ladybrown_arm.clone();
            move |color: AllianceColor| {
                if color == Skills::color() {
                    lady_brown.borrow_mut().set_state(LadyBrownState::Load);
                    return true;
                }
                false
            }
        }));

        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(0.0, -42.0))
            .params(params_move_to_point!(min_linear_speed: 0.2, early_exit_range: 1.0,))
            .call()
            .await;
        chassis.wait_until(5.0).await;
        ladybrown_arm.borrow_mut().set_state(LadyBrownState::Load);
        intake.lock().await.clear_optical_callback();
        chassis.wait_until_complete().await;
        intake.lock().await.set_velocity(-0.2);
        ladybrown_arm.borrow_mut().set_state(LadyBrownState::LoadUp);

        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(0.0, -58.0))
            .timeout(Duration::from_millis(1000))
            .params(params_turn_to!(forwards: false, min_speed: 0.3, early_exit_range: 10.0.deg(),))
            .call()
            .await;
        Rc::clone(&chassis)
            .ramsete_hybrid()
            .target(RAMSETETarget::pose(0.0, -58.0, 180.0.hdg_deg()))
            .timeout(Duration::from_millis(1000))
            .params(params_ramsete_h!(forwards: false, max_linear_speed: 0.6))
            .call()
            .await;
        vexide::time::sleep(Duration::from_millis(100)).await;
        intake.lock().await.spin();
        chassis.wait_until_complete().await;
        ladybrown_arm
            .borrow_mut()
            .set_state(LadyBrownState::Neutral);
        vexide::time::sleep(Duration::from_millis(500)).await;
        Rc::clone(&chassis)
            .move_relative()
            .distance(8.0)
            .params(
                MoveRelativeParameters::builder()
                    .early_exit_range(2.0)
                    .min_linear_speed(0.4)
                    .build(),
            )
            .call()
            .await;

        println!("{}", Skills::color().get_name());
    }
}
