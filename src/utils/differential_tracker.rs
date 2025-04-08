use alloc::collections::VecDeque;

use vexide::time::Instant;

/// Tracks derivatives of a value over time.
#[derive(Clone)]
pub struct DifferentialTracker {
    n: usize,
    data: VecDeque<(Instant, f64)>,
    cumulative_integral: f64,
}

impl DifferentialTracker {
    /// Creates a new `DifferentialTracker` with a specified size.
    /// The size must be at least 1, which is not meaningful since you can only get the value.
    /// A derivative would require an n of at least 2.
    pub fn new(n: usize) -> Self {
        assert!(n >= 1, "n must be at least 1");
        Self {
            n,
            data: VecDeque::with_capacity(n),
            cumulative_integral: 0.0,
        }
    }

    pub fn space_size(&self) -> usize {
        self.n
    }

    pub fn integral(&self) -> f64 {
        self.cumulative_integral
    }

    pub fn reset_integral(&mut self) {
        self.cumulative_integral = 0.0;
    }

    pub fn reset(&mut self) {
        self.data.clear();
        self.reset_integral();
    }

    /// Updates the tracker with a new value.
    /// If the tracker is full, it will remove the oldest value.
    pub fn update(&mut self, value: f64) {
        let now = Instant::now();

        // Use trapezoidal rule to calculate the integral.
        if let Some(&(previous_time, previous_value)) = self.data.back() {
            let dt = now.duration_since(previous_time).as_secs_f64() * 1000.0;
            let area = 0.5 * (previous_value + value) * dt;
            self.cumulative_integral += area;
        }

        if self.data.len() == self.n {
            self.data.pop_front();
        }
        self.data.push_back((now, value));
    }

    pub fn at_index(&self, idx: usize) -> Option<f64> {
        if idx >= self.data.len() {
            return None;
        }
        Some(self.data[idx].1)
    }

    /// Returns the kth order derivative of the last value in the tracker.
    pub fn derivative(&self, k: usize) -> Option<f64> {
        if k >= self.n {
            return None;
        }
        if k == 0 {
            return self.data.back().map(|(_, v)| *v);
        }

        Some(self.compute_derivative(k, self.data.len() - 1))
    }

    fn compute_derivative(&self, k: usize, idx: usize) -> f64 {
        if k == 0 {
            return self.data[idx].1;
        }

        let (t1, _) = self.data[idx];
        let (t0, _) = self.data[idx - 1];

        let dt = t1.duration_since(t0).as_secs_f64() * 1000.0;
        if dt == 0.0 {
            return 0.0;
        }

        let dv = self.compute_derivative(k - 1, idx) - self.compute_derivative(k - 1, idx - 1);
        dv / dt
    }
}
