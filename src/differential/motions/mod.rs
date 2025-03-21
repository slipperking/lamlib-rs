#[macro_use]
pub mod angular;
#[macro_use]
pub mod lateral;
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
    in_motion: RefCell<bool>,
    in_queue: RefCell<bool>,

    mutex: BlockingQueue,
}

impl MotionHandler {
    pub fn new() -> Self {
        Self {
            in_motion: RefCell::new(false),
            in_queue: RefCell::new(false),
            mutex: BlockingQueue::new(),
        }
    }
    pub fn in_motion(&self) -> bool {
        *self.in_motion.borrow()
    }
    pub async fn wait_for_motions_end(&self) {
        if *self.in_motion.borrow() {
            *self.in_queue.borrow_mut() = true;
        } else {
            *self.in_motion.borrow_mut() = true;
        }
        self.mutex.take().await;
    }
    pub async fn end_motion(&self) {
        {
            let mut queued_motion = self.in_queue.borrow_mut();
            *self.in_motion.borrow_mut() = *queued_motion;
            *queued_motion = false;
        }
        self.mutex.give().await;
    }

    pub async fn cancel_all_motions(&self) {
        *self.in_motion.borrow_mut() = false;
        *self.in_queue.borrow_mut() = false;
        vexide::time::sleep(Duration::from_millis(10)).await;
    }

    pub async fn cancel_motion(&self) {
        *self.in_motion.borrow_mut() = false;
        vexide::time::sleep(Duration::from_millis(10)).await;
    }
}

impl Default for MotionHandler {
    fn default() -> Self {
        Self::new()
    }
}
pub struct ExitCondition<T> {
    range: T,
    timeout: Duration,
    start_time: Option<Instant>,
    done: bool,
}

impl<T: num_traits::Float + Copy> ExitCondition<T> {
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

impl<T> ExitCondition<T> {
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

impl<T: Copy> Clone for ExitCondition<T> {
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
pub struct ExitConditionGroup<T: Copy> {
    pub exit_condition_group: Vec<ExitCondition<T>>,
}

impl<T: Copy> ExitConditionGroup<T> {
    pub fn new(exit_condition_group: Vec<ExitCondition<T>>) -> Self {
        Self {
            exit_condition_group,
        }
    }
    pub fn reset(&mut self) {
        for exit_condition in self.exit_condition_group.iter_mut() {
            exit_condition.reset();
        }
    }
}
impl<T: Copy + num_traits::Float + Copy> ExitConditionGroup<T> {
    pub fn update_all(&mut self, error: T) -> bool {
        let mut done = false;
        for exit_condition in self.exit_condition_group.iter_mut() {
            if exit_condition.update(error) {
                done = true;
            }
        }
        done
    }
}
