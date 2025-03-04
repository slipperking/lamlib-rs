#[macro_use]
pub mod math;
pub mod samplers;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AllianceColor {
    None,
    Red,
    Blue,
}
