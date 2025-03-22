pub mod chassis;
pub mod drive_curve;
#[macro_use]
pub mod motions;

#[macro_use]
pub mod pose {
    use core::ops::{Add, Sub};

    use nalgebra::{Vector2, Vector3};
    use num_traits::{AsPrimitive, Num};

    #[derive(Clone, Copy, PartialEq, Debug)]
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

    impl Pose {
        pub fn new<T: Num + AsPrimitive<f64>, U: Num + AsPrimitive<f64>, V: Num + AsPrimitive<f64>>(
            x: T,
            y: U,
            orientation: V,
        ) -> Self {
            Self {
                position: Vector2::<f64>::new(x.as_(), y.as_()),
                orientation: orientation.as_(),
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
