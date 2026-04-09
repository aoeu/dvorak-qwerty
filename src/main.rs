// dvorak.rs – Rust port of dvorak.c
//
// Reads raw events from a physical keyboard via /dev/input/…, remaps them
// from QWERTY to Dvorak (while keeping modifier-key shortcuts on their
// QWERTY positions), implements a capslock→enter→backspace→escape cycle,
// custom swaps, and a space-as-meta chord, then replays events through a
// uinput virtual device.
//
// Only external dependency: libc (for Linux syscall wrappers and types).

#![allow(clippy::too_many_arguments)]

use std::mem;
use std::os::unix::io::RawFd;
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use libc::{c_int, c_ulong, c_void, timeval, EINTR, F_GETFL, F_SETFL, O_NONBLOCK, O_RDONLY, O_WRONLY};

// ── Linux input / uinput constants ───────────────────────────────────────────

// Event types
const EV_SYN: u16 = 0x00;
const EV_KEY: u16 = 0x01;
const EV_REL: u16 = 0x02;
const EV_ABS: u16 = 0x03;
const EV_MSC: u16 = 0x04;
const EV_SW: u16 = 0x05;
const EV_MAX: u16 = 0x1f;

const SYN_REPORT: u16 = 0;

// Key codes (from linux/input-event-codes.h)
const KEY_ESC: u16 = 1;
const KEY_BACKSPACE: u16 = 14;
const KEY_MINUS: u16 = 12;
const KEY_EQUAL: u16 = 13;
const KEY_Q: u16 = 16;
const KEY_W: u16 = 17;
const KEY_E: u16 = 18;
const KEY_R: u16 = 19;
const KEY_T: u16 = 20;
const KEY_Y: u16 = 21;
const KEY_U: u16 = 22;
const KEY_I: u16 = 23;
const KEY_O: u16 = 24;
const KEY_P: u16 = 25;
const KEY_LEFTBRACE: u16 = 26;
const KEY_RIGHTBRACE: u16 = 27;
const KEY_ENTER: u16 = 28;
const KEY_LEFTCTRL: u16 = 29;
const KEY_A: u16 = 30;
const KEY_S: u16 = 31;
const KEY_D: u16 = 32;
const KEY_F: u16 = 33;
const KEY_G: u16 = 34;
const KEY_H: u16 = 35;
const KEY_J: u16 = 36;
const KEY_K: u16 = 37;
const KEY_L: u16 = 38;
const KEY_SEMICOLON: u16 = 39;
const KEY_APOSTROPHE: u16 = 40;
const KEY_Z: u16 = 44;
const KEY_X: u16 = 45;
const KEY_C: u16 = 46;
const KEY_V: u16 = 47;
const KEY_B: u16 = 48;
const KEY_N: u16 = 49;
const KEY_M: u16 = 50;
const KEY_COMMA: u16 = 51;
const KEY_DOT: u16 = 52;
const KEY_SLASH: u16 = 53;
const KEY_SPACE: u16 = 57;
const KEY_CAPSLOCK: u16 = 58;
const KEY_PAUSE: u16 = 119;
const KEY_LEFTALT: u16 = 56;
const KEY_LEFTMETA: u16 = 125;
const KEY_RIGHTCTRL: u16 = 97;
const KEY_MAX: u16 = 0x2ff;

const REL_MAX: u16 = 0x0f;
const ABS_MAX: u16 = 0x3f;
const MSC_MAX: u16 = 0x07;

const BUS_USB: u16 = 0x03;
const UINPUT_MAX_NAME_SIZE: usize = 80;

// ── ioctl request numbers ────────────────────────────────────────────────────
//
// Linux _IOC(dir, type, nr, size) = (dir<<30) | (type<<8) | nr | (size<<16)
//   _IOC_READ=2, _IOC_WRITE=1, _IOC_NONE=0
//
// All values below are computed for Linux x86_64.

// EVIOCGRAB = _IOW('E'=0x45, 0x90, int=4)
//           = (1<<30)|(0x45<<8)|0x90|(4<<16) = 0x40044590
const EVIOCGRAB: c_ulong = 0x4004_4590;

// UI_DEV_CREATE = _IO('U'=0x55, 1) = 0x5501
const UI_DEV_CREATE: c_ulong = 0x5501;

// UI_DEV_SETUP = _IOW('U', 3, uinput_setup)  sizeof(uinput_setup)=92=0x5c
//              = (1<<30)|(0x55<<8)|3|(92<<16) = 0x405c5503
const UI_DEV_SETUP: c_ulong = 0x405c_5503;

// UI_ABS_SETUP = _IOW('U', 4, uinput_abs_setup)  sizeof=28=0x1c
//              = (1<<30)|(0x55<<8)|4|(28<<16) = 0x401c5504
const UI_ABS_SETUP: c_ulong = 0x401c_5504;

// UI_SET_*BIT = _IOW('U', 100..104, int=4)
//             = (1<<30)|(0x55<<8)|nr|(4<<16)
const UI_SET_EVBIT: c_ulong = 0x4004_5564; // nr=100=0x64
const UI_SET_KEYBIT: c_ulong = 0x4004_5565; // nr=101
const UI_SET_RELBIT: c_ulong = 0x4004_5566; // nr=102
const UI_SET_ABSBIT: c_ulong = 0x4004_5567; // nr=103
const UI_SET_MSCBIT: c_ulong = 0x4004_5568; // nr=104

/// EVIOCGNAME(len) = _IOC(2, 'E', 6, len)
fn eviocgname(len: usize) -> c_ulong {
    ((2u64 << 30) | (0x45u64 << 8) | 6 | ((len as u64) << 16)) as c_ulong
}

/// EVIOCGBIT(ev, len) = _IOC(2, 'E', 0x20+ev, len)
fn eviocgbit(ev: u32, len: usize) -> c_ulong {
    ((2u64 << 30) | (0x45u64 << 8) | (0x20 + ev as u64) | ((len as u64) << 16)) as c_ulong
}

/// EVIOCGABS(abs) = _IOC(2, 'E', 0x40+abs, sizeof(InputAbsinfo)=24)
fn eviocgabs(abs: u32) -> c_ulong {
    ((2u64 << 30) | (0x45u64 << 8) | (0x40 + abs as u64) | (24u64 << 16)) as c_ulong
}

// ── Structures (must match the C ABI exactly) ─────────────────────────────────

/// Mirrors `struct input_event` from <linux/input.h>.
/// On 64-bit Linux: time=16 B, type+code=4 B, value=4 B → 24 B total.
#[repr(C)]
struct InputEvent {
    time: timeval,
    ev_type: u16,
    code: u16,
    value: i32,
}

/// Mirrors `struct input_id` from <linux/input.h>.
#[repr(C)]
#[derive(Copy, Clone, Default)]
struct InputId {
    bustype: u16,
    vendor: u16,
    product: u16,
    version: u16,
}

/// Mirrors `struct uinput_setup` from <linux/uinput.h>.
/// sizeof = 8 (id) + 80 (name) + 4 (ff_effects_max) = 92 bytes.
#[repr(C)]
struct UinputSetup {
    id: InputId,
    name: [u8; UINPUT_MAX_NAME_SIZE],
    ff_effects_max: u32,
}

/// Mirrors `struct input_absinfo` from <linux/input.h>. 6 × i32 = 24 bytes.
#[repr(C)]
#[derive(Copy, Clone)]
struct InputAbsinfo {
    value: i32,
    minimum: i32,
    maximum: i32,
    fuzz: i32,
    flat: i32,
    resolution: i32,
}

/// Mirrors `struct uinput_abs_setup` from <linux/uinput.h>.
/// sizeof = 2 (code) + 2 (padding) + 24 (absinfo) = 28 bytes.
#[repr(C)]
struct UinputAbsSetup {
    code: u16,
    _pad: u16,
    absinfo: InputAbsinfo,
}

// ── Signal handling ───────────────────────────────────────────────────────────

static KEEP_RUNNING: AtomicBool = AtomicBool::new(true);

extern "C" fn sig_handler(_sig: c_int) {
    // AtomicBool::store is async-signal-safe (lock-free).
    KEEP_RUNNING.store(false, Ordering::Relaxed);
}

// ── Emit helper ───────────────────────────────────────────────────────────────

fn emit(fd: RawFd, ev_type: u16, code: u16, value: i32, time: timeval) {
    let ev = InputEvent { time, ev_type, code, value };
    // SAFETY: `ev` is a valid, fully-initialised repr(C) struct; write() is
    // inherently unsafe but there is no alternative for uinput I/O.
    unsafe {
        libc::write(
            fd,
            &ev as *const InputEvent as *const c_void,
            mem::size_of::<InputEvent>(),
        );
    }
}

// ── Key-mapping functions ─────────────────────────────────────────────────────

fn modifier_bit(key: u16) -> i32 {
    match key {
        KEY_LEFTCTRL => 1,
        KEY_RIGHTCTRL => 2,
        KEY_LEFTALT => 4,
        KEY_LEFTMETA => 8,
        _ => 0,
    }
}

/// Maps a physical QWERTY key to the keycode that produces the correct
/// Dvorak character when the OS layout is set to QWERTY.
fn dvorak_remap(key: u16) -> u16 {
    match key {
        KEY_MINUS => KEY_LEFTBRACE,
        KEY_EQUAL => KEY_RIGHTBRACE,
        KEY_Q => KEY_APOSTROPHE,
        KEY_W => KEY_COMMA,
        KEY_E => KEY_DOT,
        KEY_R => KEY_P,
        KEY_T => KEY_Y,
        KEY_Y => KEY_F,
        KEY_U => KEY_G,
        KEY_I => KEY_C,
        KEY_O => KEY_R,
        KEY_P => KEY_L,
        KEY_LEFTBRACE => KEY_SLASH,
        KEY_RIGHTBRACE => KEY_EQUAL,
        KEY_A => KEY_A,
        KEY_S => KEY_O,
        KEY_D => KEY_E,
        KEY_F => KEY_U,
        KEY_G => KEY_I,
        KEY_H => KEY_D,
        KEY_J => KEY_H,
        KEY_K => KEY_T,
        KEY_L => KEY_N,
        KEY_SEMICOLON => KEY_S,
        KEY_APOSTROPHE => KEY_MINUS,
        KEY_Z => KEY_SEMICOLON,
        KEY_X => KEY_Q,
        KEY_C => KEY_J,
        KEY_V => KEY_K,
        KEY_B => KEY_X,
        KEY_N => KEY_B,
        KEY_M => KEY_M,
        KEY_COMMA => KEY_W,
        KEY_DOT => KEY_V,
        KEY_SLASH => KEY_Z,
        k => k,
    }
}

/// Always-on key cycle: capslock → enter → backspace → escape → capslock.
/// Applied before anything else; bypasses dvorak translation entirely.
fn custom_cycle(key: u16) -> u16 {
    match key {
        KEY_CAPSLOCK => KEY_ENTER,
        KEY_ENTER => KEY_BACKSPACE,
        KEY_BACKSPACE => KEY_ESC,
        KEY_ESC => KEY_CAPSLOCK,
        k => k,
    }
}

/// Swaps applied during normal typing (no modifier), before dvorak translation.
fn custom_swap(key: u16) -> u16 {
    match key {
        KEY_Q => KEY_Z,
        KEY_Z => KEY_Q,
        KEY_LEFTBRACE => KEY_SLASH,
        KEY_SLASH => KEY_LEFTBRACE,
        k => k,
    }
}

// ── Space-as-meta state machine ───────────────────────────────────────────────
// Based on https://gitlab.com/interception/linux/plugins/space2meta

const SM_DELAY_US: u64 = 20_000;

#[derive(Debug, Clone, Copy, PartialEq)]
enum SmState {
    Start,
    SpaceHeld,
    KeyHeld,
    SpaceIsMeta,
}

struct Sm {
    state: SmState,
    key_held_time: timeval,
    key_held_raw: u16,    // physical keycode — used for meta chord emission
    key_held_mapped: u16, // dvorak keycode   — used for tap/character emission
}

impl Sm {
    fn new() -> Self {
        // SAFETY: timeval is a plain C struct; zeroing it is valid.
        Sm {
            state: SmState::Start,
            key_held_time: unsafe { mem::zeroed() },
            key_held_raw: 0,
            key_held_mapped: 0,
        }
    }
}

fn sm_sleep() {
    thread::sleep(Duration::from_micros(SM_DELAY_US));
}

/// The space-as-meta state machine.  Receives both the raw physical keycode
/// (`raw_code`) and the already-remapped dvorak keycode (`mapped_code`).
/// Meta-chord paths emit `raw_code` (QWERTY positions); tap/character paths
/// emit `mapped_code` (Dvorak output).
fn sm_emit(
    sm: &mut Sm,
    out_fd: RawFd,
    ev_type: u16,
    raw_code: u16,
    mapped_code: u16,
    value: i32,
    time: timeval,
) {
    match sm.state {
        SmState::Start => {
            if ev_type == EV_KEY && raw_code == KEY_SPACE && value == 1 {
                sm.state = SmState::SpaceHeld;
                return; // buffer space; wait to see what follows
            }
            emit(out_fd, ev_type, mapped_code, value, time);
        }

        SmState::SpaceHeld => {
            if ev_type == EV_KEY && raw_code == KEY_SPACE {
                if value != 0 {
                    return; // suppress repeat
                }
                // space released alone → emit a real space tap
                emit(out_fd, EV_KEY, KEY_SPACE, 1, time);
                emit(out_fd, EV_SYN, SYN_REPORT, 0, time);
                sm_sleep();
                emit(out_fd, EV_KEY, KEY_SPACE, 0, time);
                sm.state = SmState::Start;
                return;
            }
            if ev_type == EV_KEY && value == 1 {
                // another key pressed — buffer it; resolve on next event
                sm.key_held_raw = raw_code;
                sm.key_held_mapped = mapped_code;
                sm.key_held_time = time;
                sm.state = SmState::KeyHeld;
                return;
            }
            if ev_type == EV_KEY {
                // release/repeat of a key held before space was pressed
                emit(out_fd, EV_KEY, KEY_SPACE, 1, time);
                emit(out_fd, EV_SYN, SYN_REPORT, 0, time);
                sm_sleep();
                emit(out_fd, ev_type, mapped_code, value, time);
                sm.state = SmState::Start;
                return;
            }
            if ev_type == EV_REL || ev_type == EV_ABS {
                // mouse/joystick movement while space held → activate meta
                emit(out_fd, EV_KEY, KEY_LEFTMETA, 1, time);
                emit(out_fd, EV_SYN, SYN_REPORT, 0, time);
                sm_sleep();
                emit(out_fd, ev_type, raw_code, value, time);
                sm.state = SmState::SpaceIsMeta;
                return;
            }
            // EV_SYN or other — pass through without changing state
            emit(out_fd, ev_type, mapped_code, value, time);
        }

        SmState::KeyHeld => {
            // suppress repeats of space or the buffered key while deciding
            if ev_type == EV_KEY
                && (raw_code == KEY_SPACE || raw_code == sm.key_held_raw)
                && value != 0
            {
                return;
            }
            if ev_type == EV_KEY && raw_code == KEY_SPACE {
                // space released — was just a prefix, not meta; emit tap
                let kht = sm.key_held_time;
                let khm = sm.key_held_mapped;
                emit(out_fd, EV_KEY, KEY_SPACE, 1, kht);
                emit(out_fd, EV_SYN, SYN_REPORT, 0, time);
                sm_sleep();
                emit(out_fd, EV_KEY, khm, 1, kht);
                emit(out_fd, EV_SYN, SYN_REPORT, 0, time);
                sm_sleep();
                emit(out_fd, ev_type, mapped_code, value, time);
                sm.state = SmState::Start;
            } else if ev_type == EV_KEY || ev_type == EV_REL || ev_type == EV_ABS {
                // another event — commit: space becomes meta, use raw keycodes
                let kht = sm.key_held_time;
                let khr = sm.key_held_raw;
                emit(out_fd, EV_KEY, KEY_LEFTMETA, 1, kht);
                emit(out_fd, EV_SYN, SYN_REPORT, 0, time);
                sm_sleep();
                emit(out_fd, EV_KEY, khr, 1, kht);
                emit(out_fd, EV_SYN, SYN_REPORT, 0, time);
                sm_sleep();
                emit(out_fd, ev_type, raw_code, value, time);
                sm.state = SmState::SpaceIsMeta;
            } else {
                // EV_SYN or other — pass through without changing state
                emit(out_fd, ev_type, mapped_code, value, time);
            }
        }

        SmState::SpaceIsMeta => {
            if ev_type == EV_KEY && raw_code == KEY_SPACE {
                if value == 0 {
                    sm.state = SmState::Start;
                }
                emit(out_fd, EV_KEY, KEY_LEFTMETA, value, time);
                return;
            }
            // In meta mode all keys use raw (QWERTY) positions
            emit(out_fd, ev_type, raw_code, value, time);
        }
    }
}

// ── Capability helpers ────────────────────────────────────────────────────────

fn has_event_type(array_bit_ev: &[u32], event_type: u16) -> bool {
    (array_bit_ev[(event_type / 32) as usize] & (1u32 << (event_type % 32))) != 0
}

/// Iterates every bit set in `array_bit` (indices 0..<max_val) and calls the
/// appropriate UI_SET_*BIT ioctl on `out_fd`.  For ABS axes the absinfo is
/// read from `fdi` first.
fn setup_event_type(
    fdi: RawFd,
    out_fd: RawFd,
    event_type: c_ulong,
    max_val: u16,
    array_bit: &[u32],
) -> bool {
    for i in 0..max_val {
        if (array_bit[(i / 32) as usize] & (1u32 << (i % 32))) == 0 {
            continue;
        }
        // SAFETY: ioctl is inherently unsafe; all arguments are valid.
        let ret: c_int = unsafe {
            match event_type {
                UI_SET_EVBIT => libc::ioctl(out_fd, UI_SET_EVBIT, i as c_int),
                UI_SET_KEYBIT => libc::ioctl(out_fd, UI_SET_KEYBIT, i as c_int),
                UI_SET_RELBIT => libc::ioctl(out_fd, UI_SET_RELBIT, i as c_int),
                UI_SET_MSCBIT => libc::ioctl(out_fd, UI_SET_MSCBIT, i as c_int),
                UI_SET_ABSBIT => {
                    let mut abs_setup = UinputAbsSetup {
                        code: i,
                        _pad: 0,
                        absinfo: InputAbsinfo {
                            value: 0,
                            minimum: 0,
                            maximum: 0,
                            fuzz: 0,
                            flat: 0,
                            resolution: 0,
                        },
                    };
                    if libc::ioctl(
                        fdi,
                        eviocgabs(i as u32),
                        &mut abs_setup.absinfo as *mut InputAbsinfo,
                    ) < 0
                    {
                        eprintln!(
                            "Failed to get ABS info for axis {}: {}",
                            i,
                            std::io::Error::last_os_error()
                        );
                        continue;
                    }
                    if libc::ioctl(out_fd, UI_ABS_SETUP, &abs_setup as *const UinputAbsSetup) < 0 {
                        eprintln!(
                            "Failed to setup ABS axis {}: {}",
                            i,
                            std::io::Error::last_os_error()
                        );
                        continue;
                    }
                    libc::ioctl(out_fd, UI_SET_ABSBIT, i as c_int)
                }
                _ => -1,
            }
        };
        if ret < 0 {
            eprintln!(
                "Cannot set bit {} for event_type {:#x}: {}",
                i,
                event_type,
                std::io::Error::last_os_error()
            );
            return false;
        }
    }
    true
}

// ── Remapped-key tracking ─────────────────────────────────────────────────────

const MAX_LENGTH: usize = 8;

#[derive(Copy, Clone)]
struct KeyPair {
    original: u16,
    emitted: u16,
}

fn remapped_find(keys: &[KeyPair], original: u16) -> Option<u16> {
    keys.iter()
        .find(|k| k.original == original)
        .map(|k| k.emitted)
}

fn remapped_remove(keys: &mut Vec<KeyPair>, original: u16) -> Option<u16> {
    if let Some(pos) = keys.iter().position(|k| k.original == original) {
        let emitted = keys[pos].emitted;
        keys.remove(pos);
        Some(emitted)
    } else {
        None
    }
}

// ── Time helper ───────────────────────────────────────────────────────────────

fn current_time() -> timeval {
    // SAFETY: gettimeofday with a null timezone pointer is well-defined.
    let mut tv: timeval = unsafe { mem::zeroed() };
    unsafe { libc::gettimeofday(&mut tv, ptr::null_mut()) };
    tv
}

// ── Usage ─────────────────────────────────────────────────────────────────────

fn usage(prog: &str) {
    let basename = prog.rsplit('/').next().unwrap_or(prog);
    eprintln!("usage: {} [OPTION]", basename);
    eprintln!(
        "  -d /dev/input/by-id/…\tSpecifies which device should be captured."
    );
    eprintln!(
        "  -m STRING\t\tMatch only STRING with the USB device name.\n\
         \t\t\tSTRING can contain multiple words, separated by space."
    );
    eprintln!(
        "example: {} -d /dev/input/by-id/usb-Logitech_USB_Receiver-if02-event-kbd -m \"k750 k350\"",
        basename
    );
}

// ── main ──────────────────────────────────────────────────────────────────────

fn main() {
    // Install signal handlers without SA_RESTART so that the blocking read()
    // in the event loop returns EINTR when a signal arrives → clean exit.
    // SAFETY: sigaction is inherently unsafe; our handler only stores to an
    // AtomicBool, which is async-signal-safe.
    unsafe {
        let mut sa: libc::sigaction = mem::zeroed();
        sa.sa_sigaction = sig_handler as libc::sighandler_t;
        libc::sigemptyset(&mut sa.sa_mask);
        // sa_flags = 0: no SA_RESTART
        libc::sigaction(libc::SIGTERM, &sa, ptr::null_mut());
        libc::sigaction(libc::SIGINT, &sa, ptr::null_mut());
    }

    // ── Argument parsing ──────────────────────────────────────────────────────
    let args: Vec<String> = std::env::args().collect();
    let mut device: Option<String> = None;
    let mut match_str: Option<String> = None;
    let mut args_ok = true;

    let mut idx = 1usize;
    while idx < args.len() {
        match args[idx].as_str() {
            "-d" => {
                idx += 1;
                if idx < args.len() {
                    device = Some(args[idx].clone());
                } else {
                    args_ok = false;
                }
            }
            "-m" => {
                idx += 1;
                if idx < args.len() {
                    match_str = Some(args[idx].clone());
                } else {
                    args_ok = false;
                }
            }
            _ => {
                args_ok = false;
            }
        }
        idx += 1;
    }

    let device = match (args_ok, device) {
        (true, Some(d)) => d,
        _ => {
            usage(&args[0]);
            eprintln!("Error: Input device not specified.");
            eprintln!(
                "Hint: Provide a valid input device, typically found under /dev/input/by-id/..."
            );
            std::process::exit(1);
        }
    };

    // ── Open input device ─────────────────────────────────────────────────────
    let dev_cstr = std::ffi::CString::new(device.as_bytes()).expect("device path has interior NUL");
    // SAFETY: open() is always unsafe; path and flags are valid.
    let fdi: RawFd = unsafe { libc::open(dev_cstr.as_ptr(), O_RDONLY) };
    if fdi < 0 {
        eprintln!(
            "Error: Failed to open device [{}]: {}.",
            device,
            std::io::Error::last_os_error()
        );
        eprintln!(
            "Hint: Check if the device path is correct and you have the necessary permissions."
        );
        std::process::exit(1);
    }

    // ── Read device name ──────────────────────────────────────────────────────
    let mut kname_buf = [0u8; UINPUT_MAX_NAME_SIZE];
    // SAFETY: ioctl fills kname_buf with at most UINPUT_MAX_NAME_SIZE-1 bytes.
    if unsafe {
        libc::ioctl(
            fdi,
            eviocgname(UINPUT_MAX_NAME_SIZE - 1),
            kname_buf.as_mut_ptr(),
        )
    } < 0
    {
        eprintln!(
            "Error: Unable to retrieve device name for [{}]: {}.",
            device,
            std::io::Error::last_os_error()
        );
        eprintln!("Hint: Verify if the device is functional and properly configured.");
        unsafe { libc::close(fdi) };
        std::process::exit(1);
    }
    let nul_pos = kname_buf
        .iter()
        .position(|&b| b == 0)
        .unwrap_or(UINPUT_MAX_NAME_SIZE);
    let keyboard_name = String::from_utf8_lossy(&kname_buf[..nul_pos]).to_string();

    // ── Skip our own virtual device ───────────────────────────────────────────
    const VIRTUAL_NAME: &str = "Virtual Dvorak Keyboard";
    if keyboard_name == VIRTUAL_NAME {
        println!(
            "Info: Skipping mapping for the device we just created: {}.",
            keyboard_name
        );
        unsafe { libc::close(fdi) };
        return;
    }

    // ── Optional match filter ─────────────────────────────────────────────────
    if let Some(ref ms) = match_str {
        let kb_lower = keyboard_name.to_lowercase();
        let mut matched = false;
        for token in ms.split_whitespace() {
            if kb_lower.contains(&token.to_lowercase()) {
                println!(
                    "Info: Found matching input: [{}] for device [{}].",
                    keyboard_name, device
                );
                matched = true;
                break;
            }
        }
        if !matched {
            eprintln!(
                "Error: Device [{}] does not match any of the specified keywords: [{}].",
                keyboard_name, ms
            );
            unsafe { libc::close(fdi) };
            std::process::exit(1);
        }
    }

    // ── Read device capabilities ──────────────────────────────────────────────
    let ev_words = (EV_MAX as usize / 32) + 1;
    let key_words = (KEY_MAX as usize / 32) + 1;
    let rel_words = (REL_MAX as usize / 32) + 1;
    let abs_words = (ABS_MAX as usize / 32) + 1;
    let msc_words = (MSC_MAX as usize / 32) + 1;

    let mut bit_ev = vec![0u32; ev_words];
    let mut bit_key = vec![0u32; key_words];
    let mut bit_rel = vec![0u32; rel_words];
    let mut bit_abs = vec![0u32; abs_words];
    let mut bit_msc = vec![0u32; msc_words];

    macro_rules! ioctl_getbit {
        ($req:expr, $buf:expr, $errmsg:literal) => {
            if unsafe { libc::ioctl(fdi, $req, $buf.as_mut_ptr()) } < 0 {
                eprintln!($errmsg, device, std::io::Error::last_os_error());
                unsafe { libc::close(fdi) };
                std::process::exit(1);
            }
        };
    }

    ioctl_getbit!(
        eviocgbit(0, ev_words * 4),
        bit_ev,
        "Error: Failed to retrieve event capabilities for device [{}]: {}."
    );

    if has_event_type(&bit_ev, EV_KEY) {
        ioctl_getbit!(
            eviocgbit(EV_KEY as u32, key_words * 4),
            bit_key,
            "Error: Failed to retrieve EV_KEY capabilities for device [{}]: {}."
        );
    }
    if has_event_type(&bit_ev, EV_REL) {
        ioctl_getbit!(
            eviocgbit(EV_REL as u32, rel_words * 4),
            bit_rel,
            "Error: Failed to retrieve EV_REL capabilities for device [{}]: {}."
        );
    }
    if has_event_type(&bit_ev, EV_ABS) {
        ioctl_getbit!(
            eviocgbit(EV_ABS as u32, abs_words * 4),
            bit_abs,
            "Error: Failed to retrieve EV_ABS capabilities for device [{}]: {}."
        );
    }
    if has_event_type(&bit_ev, EV_MSC) {
        ioctl_getbit!(
            eviocgbit(EV_MSC as u32, msc_words * 4),
            bit_msc,
            "Error: Failed to retrieve EV_MSC capabilities for device [{}]: {}."
        );
    }

    // ── Verify this is a keyboard ─────────────────────────────────────────────
    let has_key = |k: u16| (bit_key[(k / 32) as usize] & (1u32 << (k % 32))) != 0;
    if !has_key(KEY_X) || !has_key(KEY_C) || !has_key(KEY_V) {
        println!(
            "Info: Device [{}] is not recognized as a keyboard (missing essential keys).",
            device
        );
        unsafe { libc::close(fdi) };
        return;
    }

    // ── Build virtual device descriptor ──────────────────────────────────────
    let mut usetup = UinputSetup {
        id: InputId {
            bustype: BUS_USB,
            vendor: 0x1111,
            product: 0x2222,
            version: 0,
        },
        name: [0u8; UINPUT_MAX_NAME_SIZE],
        ff_effects_max: 0,
    };
    let vname = VIRTUAL_NAME.as_bytes();
    usetup.name[..vname.len()].copy_from_slice(vname);

    // ── Open /dev/uinput ──────────────────────────────────────────────────────
    // O_NONBLOCK prevents open() from blocking during device initialisation;
    // we clear it immediately so writes block rather than silently fail with
    // EAGAIN under load.
    // SAFETY: open() is always unsafe.
    let fdo: RawFd =
        unsafe { libc::open(b"/dev/uinput\0".as_ptr() as *const libc::c_char, O_WRONLY | O_NONBLOCK) };
    if fdo < 0 {
        eprintln!(
            "Error: Failed to open /dev/uinput for device [{}]: {}.",
            device,
            std::io::Error::last_os_error()
        );
        unsafe { libc::close(fdi) };
        std::process::exit(1);
    }
    unsafe {
        let flags = libc::fcntl(fdo, F_GETFL);
        libc::fcntl(fdo, F_SETFL, flags & !O_NONBLOCK);
    }

    // ── Configure and create the virtual device ───────────────────────────────
    macro_rules! ioctl_or_exit {
        ($fd:expr, $req:expr, $arg:expr, $errmsg:literal) => {
            if unsafe { libc::ioctl($fd, $req, $arg) } < 0 {
                eprintln!($errmsg, device, std::io::Error::last_os_error());
                unsafe { libc::close(fdo); libc::close(fdi) };
                std::process::exit(1);
            }
        };
    }

    ioctl_or_exit!(
        fdo,
        UI_DEV_SETUP,
        &usetup as *const UinputSetup,
        "Error: Failed to configure the virtual device for [{}]: {}."
    );

    macro_rules! setup_or_exit {
        ($ioctl:expr, $bits:expr, $max:expr, $msg:literal) => {
            if !setup_event_type(fdi, fdo, $ioctl, $max, &$bits) {
                eprintln!($msg, device, std::io::Error::last_os_error());
                unsafe { libc::close(fdo); libc::close(fdi) };
                std::process::exit(1);
            }
        };
    }

    // EV_SW (5) is the exclusive upper bound — sets bits for types 0..4
    setup_or_exit!(UI_SET_EVBIT,  bit_ev,  EV_SW,   "Cannot setup_event_type for UI_SET_EVBIT/device [{}]: {}.");
    setup_or_exit!(UI_SET_KEYBIT, bit_key, KEY_MAX,  "Cannot setup_event_type for EV_KEY/device [{}]: {}.");
    setup_or_exit!(UI_SET_RELBIT, bit_rel, REL_MAX,  "Cannot setup_event_type for EV_REL/device [{}]: {}.");
    setup_or_exit!(UI_SET_ABSBIT, bit_abs, ABS_MAX,  "Cannot setup_event_type for EV_ABS/device [{}]: {}.");
    setup_or_exit!(UI_SET_MSCBIT, bit_msc, MSC_MAX,  "Cannot setup_event_type for MSC_MAX/device [{}]: {}.");

    if unsafe { libc::ioctl(fdo, UI_DEV_CREATE) } < 0 {
        eprintln!(
            "Cannot create device: {}.",
            std::io::Error::last_os_error()
        );
        unsafe { libc::close(fdo); libc::close(fdi) };
        std::process::exit(1);
    }

    // Wait for the virtual device to be ready
    thread::sleep(Duration::from_millis(200));

    if unsafe { libc::ioctl(fdi, EVIOCGRAB, 1i32) } < 0 {
        eprintln!(
            "Cannot grab key: {}.",
            std::io::Error::last_os_error()
        );
        unsafe { libc::close(fdo); libc::close(fdi) };
        std::process::exit(1);
    }

    // ── Event loop ────────────────────────────────────────────────────────────
    let mut sm = Sm::new();
    let mut mod_state: i32 = 0;
    let mut remapping_enabled = true;
    let mut remapped_keys: Vec<KeyPair> = Vec::with_capacity(MAX_LENGTH);

    eprintln!(
        "Starting event loop with keyboard: [{}] for device [{}].",
        keyboard_name, device
    );

    loop {
        if !KEEP_RUNNING.load(Ordering::Relaxed) {
            break;
        }

        // SAFETY: read() fills a valid, correctly-sized InputEvent buffer.
        let mut ev: InputEvent = unsafe { mem::zeroed() };
        let n = unsafe {
            libc::read(
                fdi,
                &mut ev as *mut InputEvent as *mut c_void,
                mem::size_of::<InputEvent>(),
            )
        };

        if n < 0 {
            if std::io::Error::last_os_error().raw_os_error() == Some(EINTR) {
                continue;
            }
            break;
        }
        if n != mem::size_of::<InputEvent>() as isize {
            break;
        }

        // EV_REL and EV_ABS (mouse/joystick) go through the space2meta filter.
        // EV_SYN and all other non-key events pass straight through.
        if ev.ev_type != EV_KEY {
            if ev.ev_type == EV_REL || ev.ev_type == EV_ABS {
                sm_emit(&mut sm, fdo, ev.ev_type, ev.code, ev.code, ev.value, ev.time);
            } else {
                emit(fdo, ev.ev_type, ev.code, ev.value, ev.time);
            }
            continue;
        }

        // Pause key toggles remapping on/off (press only, not repeat/release).
        if ev.code == KEY_PAUSE && ev.value == 1 {
            remapping_enabled = !remapping_enabled;
            eprintln!(
                "Remapping {}.",
                if remapping_enabled { "enabled" } else { "disabled" }
            );
            // Flush any held remapped keys so nothing gets stuck on toggle.
            let now = current_time();
            for kp in &remapped_keys {
                if kp.original == 0 {
                    continue;
                }
                emit(fdo, EV_KEY, kp.emitted, 0, now);
                emit(fdo, EV_SYN, SYN_REPORT, 0, now);
            }
            remapped_keys.clear();
            mod_state = 0;
            sm = Sm::new();
            continue;
        }

        // When remapping is disabled, pass everything straight through.
        if !remapping_enabled {
            emit(fdo, ev.ev_type, ev.code, ev.value, ev.time);
            continue;
        }

        // Track modifier state.
        let mod_bit = modifier_bit(ev.code);
        if mod_bit != 0 {
            if ev.value != 0 {
                mod_state |= mod_bit;
            } else {
                mod_state &= !mod_bit;
            }
        }

        // Cycle remap (capslock/enter/backspace/escape) is always active.
        // Must be checked before repeat/release paths for consistent translation.
        let cycled = custom_cycle(ev.code);
        if cycled != ev.code {
            sm_emit(&mut sm, fdo, ev.ev_type, cycled, cycled, ev.value, ev.time);
            continue;
        }

        // Repeat — resolve from tracking array (mod state may have changed).
        if ev.value == 2 {
            let found = remapped_find(&remapped_keys, ev.code).unwrap_or(ev.code);
            sm_emit(&mut sm, fdo, ev.ev_type, ev.code, found, ev.value, ev.time);
            continue;
        }

        // Release — remove from tracking array.
        if ev.value == 0 {
            let found = remapped_remove(&mut remapped_keys, ev.code).unwrap_or(ev.code);
            sm_emit(&mut sm, fdo, ev.ev_type, ev.code, found, ev.value, ev.time);
            continue;
        }

        // From here: key press (value == 1) only.
        // Without a modifier: custom swap then full Dvorak translation.
        // With a modifier held: raw QWERTY codes pass through for shortcuts.
        let dvorak_code = if mod_state != 0 {
            ev.code
        } else {
            dvorak_remap(custom_swap(ev.code))
        };

        // Keys that translate to themselves need no tracking.
        if dvorak_code == ev.code {
            sm_emit(&mut sm, fdo, ev.ev_type, ev.code, ev.code, ev.value, ev.time);
            continue;
        }

        // Remapped key press: track original→emitted so release can find it.
        if remapped_keys.len() >= MAX_LENGTH {
            eprintln!(
                "Warning: too many simultaneous remapped keys ({}), dropping 0x{:04x}.",
                MAX_LENGTH, ev.code
            );
        } else {
            remapped_keys.push(KeyPair {
                original: ev.code,
                emitted: dvorak_code,
            });
            sm_emit(&mut sm, fdo, ev.ev_type, ev.code, dvorak_code, ev.value, ev.time);
        }
    }

    // ── Cleanup: release any keys still held when the loop exited ─────────────
    // Each key-up needs a manual EV_SYN (the physical device's EV_SYN stream
    // is no longer flowing through).
    let now = current_time();
    for kp in &remapped_keys {
        if kp.original == 0 {
            continue;
        }
        emit(fdo, EV_KEY, kp.emitted, 0, now);
        emit(fdo, EV_SYN, SYN_REPORT, 0, now);
    }

    // SAFETY: ioctl/close are always unsafe.
    unsafe {
        libc::ioctl(fdi, EVIOCGRAB, 0i32);
        libc::close(fdi);
        libc::close(fdo);
    }
}
