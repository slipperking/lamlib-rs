use alloc::rc::Rc;

use crate::Robot;

/// A function to drive away a specific distance and use the Ladybrown to score.
/// Robot should already be initialized and be touching the stake.
pub async fn run_alliance_stake(robot: &mut Robot) {
    let chassis = robot.chassis.clone();
    robot.intake.lock().await.set_velocity(-0.1);

    Rc::clone(&chassis)
        .move_relative()
        .distance(10.0)
        .call()
        .await;

    chassis.wait_until(5.0).await;
    robot
        .ladybrown_arm
        .borrow_mut()
        .set_state(crate::subsystems::ladybrown::LadyBrownState::Alliance);
}
