#![no_std]

use core::cell::RefCell;
use critical_section::{CriticalSection, Mutex};
use esp_hal::{
    macros::handler,
    timer::{AnyTimer, PeriodicTimer},
};
use esp_println::{print, println};
use rtos_trace::RtosTrace;

static ALARM0: Mutex<RefCell<Option<PeriodicTimer<'static, AnyTimer>>>> =
    Mutex::new(RefCell::new(None));

static CYCLES_100PERCENT: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

pub fn init(mut alarm: PeriodicTimer<'static, AnyTimer>) {
    critical_section::with(|cs| {
        alarm.set_interrupt_handler(handler);
        alarm.enable_interrupt(true);

        alarm.start(fugit::MicrosDurationU64::millis(500)).unwrap();

        ALARM0.borrow_ref_mut(cs).replace(alarm);

        let mut cnt = 0u32;
        // assumes 0x7e0 = 1, 0x7e1 = 1
        unsafe {
            core::arch::asm!(
            "csrrs  {0}, 0x7e2, x0",
            inout(reg) cnt,
            options(nomem, nostack),
            );
        }
        esp_hal::delay::Delay::new().delay_millis(500);
        let mut cnt2 = 0u32;
        // assumes 0x7e0 = 1, 0x7e1 = 1
        unsafe {
            core::arch::asm!(
            "csrrs  {0}, 0x7e2, x0",
            inout(reg) cnt2,
            options(nomem, nostack),
            );
        }

        *(CYCLES_100PERCENT.borrow_ref_mut(cs)) = cnt2 - cnt;
    });

    init_hud();
}

#[handler]
fn handler() {
    critical_section::with(|cs| {
        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();

        let stats = measure(cs);
        update_hud(&stats);
    });
}

// for now max 10 tasks supported
struct TaskStats {
    // maps task id to index
    task_map: [u32; 10],

    // time spent in task (micros)
    task_time: [u64; 10],

    // current task started
    started_at: u64,
    task_id: u32,
    task_map_current: usize,
}

impl TaskStats {
    const fn new() -> Self {
        Self {
            task_map: [0u32; 10],
            task_time: [0u64; 10],
            started_at: 0u64,
            task_id: 0u32,
            task_map_current: 0usize,
        }
    }

    fn on_task_exec_begin(&mut self, id: u32) {
        self.task_id = id;
        self.started_at = esp_hal::time::now().ticks();
    }

    fn on_task_exec_end(&mut self, reset: bool) {
        let idx = self.task_map.iter().position(|v| *v == self.task_id);
        let idx = if let Some(idx) = idx {
            idx
        } else {
            self.task_map[self.task_map_current] = self.task_id;
            let idx = self.task_map_current;
            self.task_map_current += 1;
            idx
        };

        if reset {
            self.task_id = 0;
        }

        self.task_time[idx] += esp_hal::time::now().ticks() - self.started_at;
    }

    fn stats(&mut self) -> (usize, [u64; 10]) {
        // if a task is currently running measure the time
        self.on_task_exec_end(false);

        let mut stats = [0u64; 10];
        stats[..].copy_from_slice(&self.task_time[..]);

        // reset captured times so we only measure per interval
        self.task_time[..].fill(0);
        // if a task is currently running adjust the time we measure for it
        self.started_at = esp_hal::time::now().ticks();

        (self.task_map_current, stats)
    }
}

static TASK_STATS: Mutex<RefCell<TaskStats>> = Mutex::new(RefCell::new(TaskStats::new()));

struct Foo;

impl RtosTrace for Foo {
    fn task_new(_id: u32) {
        // do nothing
    }

    fn task_send_info(_id: u32, _info: rtos_trace::TaskInfo) {
        // not called
    }

    fn task_terminate(_id: u32) {
        // not called
    }

    fn task_exec_begin(id: u32) {
        critical_section::with(|cs| {
            TASK_STATS.borrow_ref_mut(cs).on_task_exec_begin(id);
        });
    }

    fn task_exec_end() {
        critical_section::with(|cs| {
            TASK_STATS.borrow_ref_mut(cs).on_task_exec_end(true);
        });
    }

    fn task_ready_begin(_id: u32) {
        // ignore
    }

    fn task_ready_end(_id: u32) {
        // ignore
    }

    fn system_idle() {
        // unused
    }

    fn isr_enter() {
        // not called
    }

    fn isr_exit() {
        // not called
    }

    fn isr_exit_to_scheduler() {
        // not called
    }

    fn marker(_id: u32) {
        // not called
    }

    fn marker_begin(_id: u32) {
        // not called
    }

    fn marker_end(_id: u32) {
        // not called
    }
}

rtos_trace::global_trace! {Foo}

fn init_hud() {
    // clear screen
    print!("\x1b[2J");

    // home
    print!("\x1b[H");

    // blue background, white foreground
    print!("\x1b[44;37m");

    // clear 2 lines
    println!("\x1b[2K");
    println!("\x1b[2K");

    // define region to 3..
    print!("\x1b[3r");

    // reset style
    print!("\x1b[0m");

    // to line 3
    print!("\x1b[3H");
}

fn update_hud(stats: &Stats) {
    // hide cursor
    print!("\x1b[?25l");

    // save cursor pos
    print!("\x1b[s");

    // set cursor to top/left
    print!("\x1b[H");

    // blue background, white foreground
    print!("\x1b[44;37m");

    // clear line
    print!("\x1b[2K");

    // print something at home (top/left)

    // CPU usage
    print!("CPU {}%", stats.cpu_usage);

    // task time usage total
    print!("\x1b[10G"); // go to column 10
    print!("TASKS ");
    for usage in stats.task_usage[..stats.task_num].iter() {
        print!("{:3}.{:01}% ", usage / 10, usage % 10);
    }

    // used heap in line 2
    print!("\x1b[2;1H");
    print!(
        "Used heap {:3}.{:01}k",
        stats.used_heap / 10,
        stats.used_heap % 10
    );

    // reset style
    print!("\x1b[0m");

    // restore cursor pos
    print!("\x1b[u");

    // show cursor
    print!("\x1b[?25h");
}

struct Stats {
    /// CPU usage in percent
    cpu_usage: u8,

    /// number of active tasks
    task_num: usize,

    /// CPU usage per task in percent * 10
    task_usage: [u16; 10],

    /// Used heap in kb * 10
    used_heap: usize,
}

fn measure(cs: CriticalSection) -> Stats {
    let cycles_100percent = *(CYCLES_100PERCENT.borrow_ref(cs)) as u64;

    let mut cnt = 0u32;

    // assumes 0x7e0 = 1, 0x7e1 = 1
    unsafe {
        core::arch::asm!(
        "csrrs  {0}, 0x7e2, x0",
        inout(reg) cnt,
        options(nomem, nostack),
        );
    }

    let cpu = unsafe {
        static mut OLD: u32 = 0;
        let diff = (cnt - OLD) as u64;
        OLD = cnt;
        u8::min(100, (diff * 100u64 / cycles_100percent) as u8)
    };

    // reset after some time to not just count the usage since boot
    let (task_len, task_stats) = critical_section::with(|cs| TASK_STATS.borrow_ref_mut(cs).stats());
    // in micros
    let elapsed = 500_000;
    let mut task_usage = [0u16; 10];
    for i in 0..task_len {
        task_usage[i] = (task_stats[i] * 1000 / elapsed) as u16;
    }

    // heap usage
    let used_heap = esp_alloc::HEAP.used() * 10 / 1024;

    // is there a way to measure stack usage reliably? especially when esp-wifi's scheduler is running?

    Stats {
        cpu_usage: cpu,
        task_num: task_len,
        task_usage,
        used_heap,
    }
}
