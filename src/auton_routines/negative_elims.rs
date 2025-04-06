use alloc::{boxed::Box, rc::Rc};
use core::time::Duration;

use async_trait::async_trait;
use lamlib_rs::{
    differential::motions::{
        angular::TurnToParameters,
        linear::{MoveRelativeParameters, MoveToPointParameters},
        ramsete::RAMSETEHybridParameters,
    }, params_move_to_point, utils::{math::AngleExt, AllianceColor}
};
use nalgebra::Vector2;

use super::AutonRoutine;
use crate::{subsystems::ladybrown::LadybrownState, Robot};
pub struct RedNegativeElims;
pub struct BlueNegativeElims;
#[async_trait(?Send)]
impl AutonRoutine for RedNegativeElims {
    fn name() -> &'static str {
        "Skills"
    }

    fn color() -> AllianceColor {
        AllianceColor::Red
    }

    fn symbol() -> &'static str {
        "EM- "
    }

    async fn run(&self, robot: &mut Robot) {
        let chassis = robot.chassis.clone();
        let intake = robot.intake.clone();
        chassis.set_filter_state(false).await;
        Rc::clone(&chassis)
            .set_pose((-60, 13, 36.8698976.hdg_deg()))
            .await;
        robot
            .ladybrown_arm
            .borrow_mut()
            .set_state(LadybrownState::Alliance);
        vexide::time::sleep(Duration::from_millis(500)).await;
        Rc::clone(&chassis)
            .move_relative()
            .distance(10.0)
            .params(
                MoveRelativeParameters::builder()
                    .min_linear_speed(0.6)
                    .early_exit_range(2.0)
                    .build(),
            )
            .run_async(false)
            .call()
            .await;

        robot
            .ladybrown_arm
            .borrow_mut()
            .set_state(LadybrownState::LoadUp);
        Rc::clone(&chassis)
            .ramsete_hybrid()
            .target((-23, 23))
            .params(
                RAMSETEHybridParameters::builder()
                    .min_linear_speed(0.4)
                    .early_exit_range(12.0)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .ramsete_hybrid()
            .target((-23, 23))
            .params(
                RAMSETEHybridParameters::builder()
                    .max_linear_speed(0.5)
                    .build(),
            )
            .call()
            .await;

        while chassis
            .pose()
            .await
            .position
            .metric_distance(&Vector2::<f64>::new(-23.0, 23.0))
            > 3.0
        {
            vexide::time::sleep(Duration::from_millis(10)).await;
        }
        robot
            .ladybrown_arm
            .borrow_mut()
            .set_state(LadybrownState::FreeTimedReset);
        robot.clamp_main.set_state(true);
        Rc::clone(&chassis)
            .turn_to()
            .target((-9, 36))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .min_speed(0.2)
                    .early_exit_range(5.0.deg())
                    .build(),
            )
            .run_async(false)
            .call()
            .await;
        intake.lock().await.spin();
        Rc::clone(&chassis)
            .move_to_point()
            .target((-9, 36))
            .params(
                MoveToPointParameters::builder()
                    .forwards(false)
                    .min_linear_speed(0.1)
                    .early_exit_range(1.0)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .turn_to()
            .target(0.0.hdg_deg())
            .params(TurnToParameters::builder().forwards(false).build())
            .call()
            .await;
        Rc::clone(&chassis)
            .move_relative()
            .distance(-15.0)
            .params(
                MoveRelativeParameters::builder()
                    .min_linear_speed(0.5)
                    .early_exit_range(2.0)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .ramsete_hybrid()
            .target((-25, 47))
            .params(
                RAMSETEHybridParameters::builder()
                    .forwards(false)
                    .max_linear_speed(0.2)
                    .early_exit_range(2.0)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .ramsete_hybrid()
            .target((-9, 12, 145.0.hdg_deg()))
            .params(RAMSETEHybridParameters::builder().forwards(false).build())
            .run_async(false)
            .call()
            .await;
        robot.doinker_right.set_state(true);
        // Artificially profile for gradual acceleration after holding onto the ring.
        Rc::clone(&chassis)
            .move_relative()
            .distance(20.0)
            .params(
                MoveRelativeParameters::builder()
                    .max_linear_speed(0.5)
                    .min_linear_speed(0.2)
                    .early_exit_range(2.0)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_relative()
            .distance(40.0)
            .params(
                MoveRelativeParameters::builder()
                    .min_linear_speed(0.3)
                    .build(),
            )
            .run_async(false)
            .call()
            .await;
        robot.doinker_left.set_state(false);
        Rc::clone(&chassis)
            .move_to_point()
            .target((-13, 30))
            .params(params_move_to_point!(forwards: false))
            .call()
            .await;
    }
}

#[async_trait(?Send)]
impl AutonRoutine for BlueNegativeElims {
    fn name() -> &'static str {
        "BlueNegativeElims"
    }

    fn color() -> AllianceColor {
        AllianceColor::Blue
    }

    fn symbol() -> &'static str {
        "EM- "
    }

    async fn run(&self, robot: &mut Robot) {
        // TODO: MIRROR THIS
        let chassis = robot.chassis.clone();
        let intake = robot.intake.clone();
        chassis.set_filter_state(false).await;
        intake.lock().await.spin();
    }
}
