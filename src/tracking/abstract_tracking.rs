use nalgebra::Point3;

pub trait Tracking {
    fn position(&mut self) -> Point3<f32>;
}
