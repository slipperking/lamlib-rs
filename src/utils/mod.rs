#[macro_use]
pub mod math;
pub mod samplers;
pub mod timer;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AllianceColor {
    None,
    Red,
    Blue,
}

pub static TILE_SIZE: f64 = 23.576533; 

impl AllianceColor {
    pub fn get_name(&self) -> &'static str {
        match self {
            AllianceColor::None => "None",
            AllianceColor::Red => "Red",
            AllianceColor::Blue => "Blue",
        }
    }
    pub fn get_symbol(&self, length: usize) -> &'static str {
        let name = self.get_name();
        let length = length.min(name.len());
        self.get_name().split_at(length).0
    }
    pub fn get_quick_symbol(&self) -> &'static str {
        self.get_symbol(1)
    }
}
