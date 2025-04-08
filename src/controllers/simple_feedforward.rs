use alloc::vec::Vec;

use crate::utils::differential_tracker::DifferentialTracker;

pub struct SimpleFeedforward {
    ks: f64,
    k_derivatives: Vec<f64>,
}

impl SimpleFeedforward {
    pub fn new(ks: f64, k_derivatives: Vec<f64>) -> Self {
        Self { ks, k_derivatives }
    }
    pub fn run(&self, derivatives: DifferentialTracker) -> f64 {
        let mut cumulative_sum = self.ks * derivatives.derivative(1).unwrap_or(0.0).signum();
        for i in 1..derivatives.space_size() {
            cumulative_sum += derivatives.derivative(i).unwrap_or(0.0)
                * self.k_derivatives.get(i).unwrap_or(&0.0);
        }
        cumulative_sum
    }
}

#[macro_export]
macro_rules! simple_feedforward_closure {
    ($ff:expr) => {{
        let ff_clone = $ff;
        move |tracker: $crate::utils::differential_tracker::DifferentialTracker| {
            ff_clone.run(tracker)
        }
    }};
}

pub use simple_feedforward_closure;
