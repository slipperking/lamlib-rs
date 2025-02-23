pub trait ControllerMethods {
    fn update(&mut self, error: f32) -> f32;
    fn reset(&mut self);
}

#[macro_use]
pub mod pid;
