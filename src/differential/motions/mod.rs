#[macro_use]
pub mod angular;
#[macro_use]
pub mod linear;
#[macro_use]
pub mod boomerang;
#[macro_use]
pub mod ramsete;

use alloc::{collections::VecDeque, vec::Vec};
use core::{cell::RefCell, time::Duration};

use vexide::{sync::Mutex, time::Instant};

struct BlockingQueue {
    /// It is locked when there is a value as the first element of the queue.
    queue: Mutex<VecDeque<usize>>,
    counter: RefCell<usize>,
}

impl BlockingQueue {
    fn new() -> Self {
        Self {
            queue: Mutex::new(VecDeque::new()),
            counter: RefCell::new(0),
        }
    }

    async fn take(&self) {
        let id = {
            let mut counter = self.counter.borrow_mut();
            let id = *counter;
            *counter += 1;
            id
        };

        let mut queue_lock = self.queue.lock().await;
        queue_lock.push_back(id);

        loop {
            if queue_lock.front().copied() == Some(id) {
                return;
            }
            drop(queue_lock);
            vexide::time::sleep(Duration::from_millis(5)).await;
            queue_lock = self.queue.lock().await;
        }
    }

    async fn give(&self) {
        self.queue.lock().await.pop_front();
    }
}

pub struct MotionHandler {
    is_in_motion: RefCell<bool>,
    is_in_queue: RefCell<bool>,

    mutex: BlockingQueue,
}

impl MotionHandler {
    pub fn new() -> Self {
        Self {
            is_in_motion: RefCell::new(false),
            is_in_queue: RefCell::new(false),
            mutex: BlockingQueue::new(),
        }
    }
    pub fn is_in_motion(&self) -> bool {
        *self.is_in_motion.borrow()
    }
    pub async fn wait_for_motions_end(&self) {
        if *self.is_in_motion.borrow() {
            *self.is_in_queue.borrow_mut() = true;
        } else {
            *self.is_in_motion.borrow_mut() = true;
        }
        self.mutex.take().await;
    }
    pub async fn end_motion(&self) {
        *self.is_in_motion.borrow_mut() = *self.is_in_queue.borrow();
        *self.is_in_queue.borrow_mut() = false;

        self.mutex.give().await;
    }

    pub async fn cancel_all_motions(&self) {
        *self.is_in_motion.borrow_mut() = false;
        *self.is_in_queue.borrow_mut() = false;
        vexide::time::sleep(Duration::from_millis(10)).await;
    }

    pub async fn cancel_motion(&self) {
        *self.is_in_motion.borrow_mut() = false;
        vexide::time::sleep(Duration::from_millis(10)).await;
    }
}

impl Default for MotionHandler {
    fn default() -> Self {
        Self::new()
    }
}
pub struct Tolerance<T> {
    range: T,
    timeout: Duration,
    start_time: Option<Instant>,
    done: bool,
}

impl<T: num_traits::Float + Copy> Tolerance<T> {
    pub fn update(&mut self, error: T) -> bool {
        if error.abs() > self.range {
            self.start_time = None;
        } else if let Some(start_time) = self.start_time {
            if Instant::now() > start_time + self.timeout {
                self.done = true;
            }
        } else {
            self.start_time = Some(Instant::now());
        }
        self.done
    }
}

impl<T> Tolerance<T> {
    pub fn new(range: T, timeout: Duration) -> Self {
        Self {
            range,
            timeout,
            start_time: None,
            done: false,
        }
    }

    pub fn exit_state(&self) -> bool {
        self.done
    }

    pub fn reset(&mut self) {
        self.start_time = None;
        self.done = false;
    }
}

impl<T: Copy> Clone for Tolerance<T> {
    fn clone(&self) -> Self {
        Self {
            range: self.range,
            timeout: self.timeout,
            start_time: None,
            done: false,
        }
    }
}
#[derive(Clone)]
pub struct ToleranceGroup<T: Copy> {
    pub tolerance_group: Vec<Tolerance<T>>,
}

impl<T: Copy> ToleranceGroup<T> {
    pub fn new(tolerance_group: Vec<Tolerance<T>>) -> Self {
        Self { tolerance_group }
    }
    pub fn reset(&mut self) {
        for tolerance in self.tolerance_group.iter_mut() {
            tolerance.reset();
        }
    }
}
impl<T: Copy + num_traits::Float + Copy> ToleranceGroup<T> {
    pub fn update_all(&mut self, error: T) -> bool {
        let mut done = false;
        for tolerance in self.tolerance_group.iter_mut() {
            if tolerance.update(error) {
                done = true;
            }
        }
        done
    }
}
