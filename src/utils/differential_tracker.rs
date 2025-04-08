use alloc::collections::VecDeque;
use num_traits::{Float, Zero};
use vexide::time::Instant;

/// Tracks derivatives of a value over time.
#[derive(Clone)]
pub struct DifferentialTracker<T: Float> {
    n: usize,
    data: VecDeque<(Instant, T)>,
    cumulative_integral: T,
}

impl<T: Float + Zero> DifferentialTracker<T> {
    /// Creates a new `DifferentialTracker` with a specified size.
    /// The size must be at least 1, which is not meaningful since you can only get the value.
    /// A derivative would require an n of at least 2.
    pub fn new(n: usize) -> Self {
        assert!(n >= 1, "n must be at least 1");
        Self {
            n,
            data: VecDeque::with_capacity(n),
            cumulative_integral: T::zero(),
        }
    }

    pub fn space_size(&self) -> usize {
        self.n
    }

    pub fn integral(&self) -> T {
        self.cumulative_integral
    }

    pub fn reset_integral(&mut self) {
        self.cumulative_integral = T::zero();
    }

    pub fn reset(&mut self) {
        self.data.clear();
        self.reset_integral();
    }

    /// Updates the tracker with a new value.
    /// If the tracker is full, it will remove the oldest value.
    pub fn update(&mut self, value: T) {
        let now = Instant::now();

        // Use trapezoidal rule to calculate the integral.
        if let Some(&(previous_time, previous_value)) = self.data.back() {
            let dt = T::from(now.duration_since(previous_time).as_secs_f64() * 1000.0).unwrap();
            let area = (previous_value + value) * dt / T::from(2.0).unwrap();
            self.cumulative_integral = self.cumulative_integral + area;
        }

        if self.data.len() == self.n {
            self.data.pop_front();
        }
        self.data.push_back((now, value));
    }

    pub fn at_index(&self, idx: usize) -> Option<T> {
        if idx >= self.data.len() {
            return None;
        }
        Some(self.data[idx].1)
    }

    /// Returns the kth order derivative of the last value in the tracker.
    pub fn derivative(&self, k: usize) -> Option<T> {
        if k >= self.n {
            return None;
        }
        if k == 0 {
            return self.data.back().map(|(_, v)| *v);
        }

        Some(self.compute_derivative(k, self.data.len() - 1))
    }

    fn compute_derivative(&self, k: usize, idx: usize) -> T {
        if k == 0 {
            return self.data[idx].1;
        }

        let (t1, _) = self.data[idx];
        let (t0, _) = self.data[idx - 1];

        let dt = T::from(t1.duration_since(t0).as_secs_f64() * 1000.0).unwrap();
        if dt == T::zero() {
            return T::zero();
        }

        let dv = self.compute_derivative(k - 1, idx) - self.compute_derivative(k - 1, idx - 1);
        dv / dt
    }
}