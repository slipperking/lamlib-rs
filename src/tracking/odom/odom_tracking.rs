use alloc::{rc::Rc, vec::Vec};

use nalgebra::Point3;
use vexide::{
    core::sync::Mutex,
    devices::smart::imu::InertialError,
    prelude::{InertialSensor, Task},
};

use super::odom_wheels::OdomWheel;
use crate::tracking::abstract_tracking::Tracking;
pub struct OdomInertial {
    /// A mutex containing the inertial sensor.
    inertial: Mutex<InertialSensor>,

    /// For imu scaling.
    /// Use larger values to make the imu return a higher value.
    scalar: f64,

    /// How much to prioritize this sensor's output.
    /// Values of all inertial weights are normalized to 1.
    weight: f64,
}
pub struct OdomSensors {
    imus: Vec<Rc<OdomInertial>>,
    horizontals: Vec<Rc<OdomWheel>>,
    verticals: Vec<Rc<OdomWheel>>,

    /// How much imu weighted rotation affects total rotation.
    /// Values of all weights are normalized to 1.
    imu_angle_weight: f64,

    /// How much horizontals weighted rotation affects total rotation.
    /// Values of all weights are normalized to 1.
    horizontal_angle_weight: f64,

    /// How much verticals weighted rotation affects total rotation.
    /// Values of all weights are normalized to 1.
    vertical_angle_weight: f64,
}
pub struct OdomTracking {
    task: Task<()>,
    sensors: Rc<OdomSensors>,
    tracked_pose: Point3<f64>,
    prev_imu_angles: Vec<Result<f64, InertialError>>,
    prev_horizontal_distances: Vec<f64>,
    prev_vertical_distances: Vec<f64>,
}
impl OdomTracking {
    async fn update(&mut self) {
        let mut imu_angles: Vec<Result<f64, InertialError>> = Vec::new();
        let mut delta_imu_angles: Vec<Option<f64>> = Vec::new();
        let mut delta_imu_weights: Vec<Option<f64>> = Vec::new();
        for imu in &self.sensors.imus {
            let imu_lock = imu.inertial.lock().await;
            let rotation = imu_lock.rotation().map(|value| {
                core::f64::consts::FRAC_PI_2 - f64::to_radians(value * imu.scalar)
            });
            imu_angles.push(rotation);
        }
        for (new, (prev, imu)) in imu_angles
            .iter()
            .zip(self.prev_imu_angles.iter().zip(&self.sensors.imus))
        {
            match (new, prev) {
                (Ok(new_val), Ok(prev_val)) => {
                    delta_imu_angles.push(Some(new_val - prev_val));
                    delta_imu_weights.push(Some(imu.weight));
                }
                _ => {
                    delta_imu_angles.push(None);
                    delta_imu_weights.push(None);
                }
            }
        }
        let delta_imu_total_weight: f64 = delta_imu_weights.iter().filter_map(|w| *w).sum();
        let delta_imu_angle: f64 = if delta_imu_total_weight > 0.0 {
            delta_imu_angles
                .iter()
                .zip(&delta_imu_weights)
                .filter_map(|(delta, weight)| match (delta, weight) {
                    (Some(d), Some(w)) => Some(d * w),
                    _ => None,
                })
                .sum::<f64>()
                / delta_imu_total_weight
        } else {
            0.0
        };
        self.prev_imu_angles = imu_angles;

        let horizontal_distances: Vec<Option<f64>> = self
            .sensors
            .horizontals
            .iter()
            .map(|tracking_wheel| tracking_wheel.distance_traveled())
            .collect();
        let vertical_distances: Vec<Option<f64>> = self
            .sensors
            .verticals
            .iter()
            .map(|tracking_wheel| tracking_wheel.distance_traveled())
            .collect();
    }
}

impl Tracking for OdomTracking {
    fn position(&mut self) -> Point3<f64> {
        self.tracked_pose
    }
    async fn init(&mut self) {
        self.sensors
            .horizontals
            .iter()
            .for_each(|tracking_wheel| tracking_wheel.init());
        self.sensors
            .verticals
            .iter()
            .for_each(|tracking_wheel| tracking_wheel.init());
        for imu in &self.sensors.imus {
            let imu_lock_value = &mut imu.inertial.lock().await;
            let calibration_result_1 = imu_lock_value.calibrate().await;
            match &calibration_result_1 {
                Ok(_) => {}
                Err(_) => {
                    let _ = imu_lock_value.calibrate().await;
                    let _ = imu_lock_value.set_rotation(0.0);
                }
            }
        }
    }
}
