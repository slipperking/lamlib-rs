pub mod chassis;
pub mod drive_curve;
pub mod motions;
#[macro_use]
pub mod pose {
    use nalgebra::{Vector2, Vector3};
    use num_traits::AsPrimitive;

    #[derive(Clone, Copy, PartialEq)]
    pub struct Pose {
        pub position: Vector2<f64>,
        pub orientation: f64,
    }

    #[macro_export]
    macro_rules! pose_radians {
        (
            x => $x:expr;
            y => $y:expr;
            orientation => $orientation:expr;
            $(standard => $standard:expr;)?
        ) => {
            {
                let standard = true;
                $(let standard = $standard;)?

                differential::pose::Pose::new(
                    $x,
                    $y,
                    if standard {
                        $orientation
                    } else {
                        core::f64::consts::FRAC_PI_2 - $orientation
                    },
                )
            }
        }
    }
    #[macro_export]
    macro_rules! pose_degrees {
        (
            x => $x:expr;
            y => $y:expr;
            orientation => $orientation:expr;
            $(standard => $standard:expr;)?
        ) => {
            {
                let orientation = ($orientation as f64).to_radians();
                let standard = true;
                $(let standard = $standard;)?

                differential::pose::Pose::new(
                    $x,
                    $y,
                    if standard {
                        orientation
                    } else {
                        core::f64::consts::FRAC_PI_2 - orientation
                    },
                )
            }
        }
    }
    pub use pose_degrees;
    pub use pose_radians;
    impl Pose {
        pub fn new(x: f64, y: f64, orientation: f64) -> Self {
            Self {
                position: Vector2::new(x, y),
                orientation,
            }
        }
        pub fn distance_to(&self, pose: &Self) -> f64 {
            self.position.metric_distance(&pose.position)
        }
    }
    impl<T: AsPrimitive<f64>> From<Vector3<T>> for Pose {
        fn from(vector: Vector3<T>) -> Self {
            Pose::new(T::as_(vector[0]), T::as_(vector[1]), T::as_(vector[2]))
        }
    }
    impl<T: num::FromPrimitive + core::default::Default> From<Pose> for Vector3<T> {
        fn from(pose: Pose) -> Self {
            Vector3::new(
                T::from_f64(pose.position.x).unwrap_or_default(),
                T::from_f64(pose.position.y).unwrap_or_default(),
                T::from_f64(pose.orientation).unwrap_or_default(),
            )
        }
    }
}
