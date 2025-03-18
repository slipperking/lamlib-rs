pub mod chassis;
pub mod drive_curve;
pub mod motions;
#[macro_use]
pub mod pose {
    use core::ops::{Add, Sub};

    use nalgebra::{Vector2, Vector3};
    use num_traits::AsPrimitive;

    #[derive(Clone, Copy, PartialEq)]
    pub struct Pose {
        pub position: Vector2<f64>,
        pub orientation: f64,
    }

    impl Sub for Pose {
        type Output = Self;

        fn sub(self, rhs: Self) -> Self {
            Self {
                position: self.position - rhs.position,
                orientation: self.orientation - rhs.orientation,
            }
        }
    }

    impl Add for Pose {
        type Output = Self;

        fn add(self, rhs: Self) -> Self {
            Self {
                position: self.position + rhs.position,
                orientation: self.orientation + rhs.orientation,
            }
        }
    }

    #[macro_export]
    macro_rules! angle {
        (
            $(degrees: $degrees:expr,)?
            $(radians: $radians:expr,)?
            $(standard: $standard:expr,)?
        ) => {
            #[allow(unused_mut, unused_assignments)]
            {
                let mut degrees: Option<f64> = None;
                let mut radians: Option<f64> = None;
                let mut standard = false;
                $(degrees = Some($degrees);)?
                $(radians = Some($radians);)?
                $(standard = $standard;)?

                if let Some(degrees) = degrees {
                    if standard {
                        degrees.to_radians()
                    }
                    else {
                        (90.0 - degrees).to_radians()
                    }
                }
                else if let Some(radians) = radians {
                    if standard {
                        radians
                    }
                    else {
                        core::f64::consts::FRAC_PI_2 - radians
                    }
                }
                else {
                    0.0
                }
            }
        };
    }
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
    impl<T: num_traits::FromPrimitive + core::default::Default> From<Pose> for Vector3<T> {
        fn from(pose: Pose) -> Self {
            Vector3::new(
                T::from_f64(pose.position.x).unwrap_or_default(),
                T::from_f64(pose.position.y).unwrap_or_default(),
                T::from_f64(pose.orientation).unwrap_or_default(),
            )
        }
    }
}
