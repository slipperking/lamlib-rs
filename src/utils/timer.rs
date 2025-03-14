use core::time::Duration;
use vexide::time::Instant;

/// Tracks elapsed time and manages time-based delays.
///
/// # Examples
/// ```
/// use crate::utils::timer::Timer;
/// use core::time::Duration;
/// let timer = Timer::new(Duration::from_secs(5));
/// ```
#[derive(Clone)]
pub struct Timer {
    period: Duration,
    elapsed_duration: Duration,
    previous_instant: Instant,
    paused: bool,
}

impl Timer {
    /// Creates a new timer with the specified duration.
    pub fn new(period: Duration) -> Self {
        Self {
            period,
            elapsed_duration: Duration::ZERO,
            previous_instant: Instant::now(),
            paused: false,
        }
    }

    /// Returns the timer's configured period.
    pub fn period(&self) -> Duration {
        self.period
    }

    /// Returns the remaining time until the timer expires.
    ///
    /// Updates the internal elapsed time before calculation.
    pub fn remaining_time(&mut self) -> Duration {
        self.update();
        self.period.saturating_sub(self.elapsed_duration)
    }

    /// Returns the elapsed time since the timer started.
    ///
    /// Updates the internal elapsed time before calculation.
    pub fn elapsed_time(&mut self) -> Duration {
        self.update();
        self.elapsed_duration
    }

    /// Checks if the timer has completed its period.
    pub fn is_done(&mut self) -> bool {
        self.update();
        self.period.saturating_sub(self.elapsed_duration) == Duration::ZERO
    }

    /// Checks if the timer is paused.
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Pauses the timer, freezing elapsed time.
    pub fn pause(&mut self) {
        if !self.paused {
            self.update();
        }
        self.paused = true;
    }

    /// Resumes the timer if paused.
    pub fn resume(&mut self) {
        if self.paused {
            self.previous_instant = Instant::now();
        }
        self.paused = false;
    }

    /// Resets the timer to zero elapsed time.
    pub fn reset(&mut self) {
        self.elapsed_duration = Duration::ZERO;
        self.previous_instant = Instant::now();
    }

    /// Updates the timer's period and resets it.
    pub fn set_period(&mut self, period: Duration) {
        self.period = period;
        self.reset();
    }

    /// Asynchronously waits until the timer completes.
    ///
    /// # Examples
    /// ```
    /// # async fn example() {
    /// use crate::timer::Timer;
    /// use core::time::Duration;
    /// let mut timer = Timer::new(Duration::from_secs(1));
    /// timer.wait_until_complete().await;
    /// # }
    /// ```
    pub async fn wait_until_complete(&mut self) {
        loop {
            vexide::time::sleep(Duration::from_millis(5)).await;
            if self.is_done() {
                break;
            }
        }
    }

    fn update(&mut self) {
        let current_instant = Instant::now();
        if !self.paused {
            self.elapsed_duration += current_instant - self.previous_instant;
        }
        self.previous_instant = current_instant;
    }
}