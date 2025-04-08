#[macro_use]
pub mod math;
pub mod samplers;
pub mod timer;
pub mod differential_tracker;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum AllianceColor {
    None,
    Red,
    Blue,
}

pub static TILE_SIZE: f64 = 23.576533;
pub static FIELD_WALL: f32 = 70.20462;

impl AllianceColor {
    pub fn name(&self) -> &'static str {
        match self {
            AllianceColor::None => "None",
            AllianceColor::Red => "Red",
            AllianceColor::Blue => "Blue",
        }
    }
    pub fn symbol(&self, length: usize) -> &'static str {
        let name = self.name();
        let length = length.min(name.len());
        self.name().split_at(length).0
    }
    pub fn quick_symbol(&self) -> &'static str {
        self.symbol(1)
    }
    pub fn opponent(&self) -> AllianceColor {
        match self {
            AllianceColor::None => AllianceColor::None,
            AllianceColor::Red => AllianceColor::Blue,
            AllianceColor::Blue => AllianceColor::Red,
        }
    }
}

impl core::fmt::Display for AllianceColor {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.name())
    }
}
