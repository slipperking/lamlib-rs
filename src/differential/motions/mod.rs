pub mod boomerang;
pub mod ramsete;
use alloc::{collections::VecDeque, rc::Rc};
use core::{cell::RefCell, time::Duration};

use vexide::{sync::Mutex, time::Instant};

struct BlockingQueue {
    /// It is locked when there is a value as the first element of the queue.
    queue: Mutex<VecDeque<usize>>,
    counter: RefCell<usize>,
}

impl BlockingQueue {
    fn new() -> Rc<Self> {
        Rc::new(Self {
            queue: Mutex::new(VecDeque::new()),
            counter: RefCell::new(0),
        })
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

    mutex: Rc<BlockingQueue>,
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
        let mutex;
        {
            if *self.in_motion.borrow() {
                *self.in_queue.borrow_mut() = true;
            } else {
                *self.in_motion.borrow_mut() = true;
            }
            mutex = self.mutex.clone();
        }
        mutex.take().await;
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
