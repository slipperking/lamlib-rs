use alloc::{
    borrow::ToOwned, boxed::Box, collections::BTreeMap, string::{String, ToString}
};
use core::{any::type_name, fmt::Write as _, time::Duration};

use log::{Level, LevelFilter, Metadata, Record};
use serde::Serialize;
use serde_json::Value;
use vexide::{
    io::{stdout, Stdout, StdoutLock, Write},
    prelude::*,
    sync::{LazyLock, Mutex},
    time::Instant,
};

extern crate alloc;

static LOG_BUFFER: Mutex<String> = Mutex::new(String::new());
static DATA_UPDATES: LazyLock<Mutex<BTreeMap<String, Value>>> = LazyLock::new(Mutex::default);

#[derive(Debug, Serialize)]
struct Message {
    data: BTreeMap<String, Value>,
    now_sec: f64,
}

struct XYVLogger;

impl log::Log for XYVLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Debug
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut buffer = LOG_BUFFER
                .try_lock()
                .expect("the log buffer should be unlocked");
            writeln!(buffer, "{} - {}", record.level(), record.args()).unwrap();
        }
    }

    fn flush(&self) {}
}

fn free_serial_buffer_bytes(_stdout: &mut StdoutLock) -> usize {
    unsafe { vex_sdk::vexSerialWriteFree(1).try_into().unwrap() }
}

pub fn record_output<T: Serialize>(key: impl Into<String>, data: T) {
    if let Err(err) = try_record_output(key, data) {
        log::error!("Failed to log a {}: {}", type_name::<T>(), err);
    }
}

pub fn try_record_output(
    key: impl Into<String>,
    data: impl Serialize,
) -> Result<(), serde_json::Error> {
    let value = serde_json::to_value(data)?;
    let mut updates = DATA_UPDATES
        .try_lock()
        .expect("the data updates map should be unlocked");
    updates.insert(key.into(), value);
    Ok(())
}

fn flush(stdout: &mut StdoutLock, init_instant: Instant, last_flush_instant: &mut Option<Instant>) {
    let mut updates = DATA_UPDATES
        .try_lock()
        .expect("the data updates map should be unlocked");
    let mut logs = LOG_BUFFER
        .try_lock()
        .expect("the log buffer should be unlocked");
    let now = Instant::now();

    // Skipping flushing if there's nothing to report helps keep serial traffic lower.

    let has_new_data = !logs.is_empty() || updates.keys().len() > 0;
    let needs_heartbeat = if let Some(last_flush) = last_flush_instant {
        now.duration_since(*last_flush) > Duration::from_millis(400)
    } else {
        true
    };

    if !has_new_data && !needs_heartbeat {
        return;
    }

    *last_flush_instant = Some(now);

    // There's no need to waste space on an empty console string.
    if !logs.is_empty() {
        updates.insert("/Console".to_string(), Value::String(logs.to_owned()));
    }

    let msg = Message {
        data: updates.clone(),
        now_sec: now.duration_since(init_instant).as_secs_f64(),
    };

    let mut msg_data = serde_json::to_vec(&msg).expect("the xyv message should be serializable");
    writeln!(msg_data).unwrap();

    if msg_data.len() > Stdout::INTERNAL_BUFFER_SIZE {
        unimplemented!("xyv does not support packets over 2048 bytes");
    } else if msg_data.len() <= free_serial_buffer_bytes(stdout) {
        logs.clear();
        updates.clear();

        stdout.write_all(&msg_data).unwrap();
    }
}

pub fn init_logger() {
    let init_instant = Instant::now();
    let mut last_flush_instant = None;

    log::set_logger(Box::leak(Box::new(XYVLogger))).expect("log implementation should not be set");
    log::set_max_level(LevelFilter::Debug);

    spawn(async move {
        loop {
            flush(&mut stdout().lock().await, init_instant, &mut last_flush_instant);
            sleep(Duration::from_millis(20)).await;
        }
    })
    .detach();
}
