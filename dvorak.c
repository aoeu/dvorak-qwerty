#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/uinput.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/time.h>

//a key combination has a maximum amount of 8 characters. That should be enough.
#define MAX_LENGTH 8

static int fdi = -1;
static int fdo = -1;
static volatile sig_atomic_t keep_running = 1;
static void sig_handler(int _sig) {
    (void)_sig;
    keep_running = 0;
    // Cleanup (ungrab + close) happens in main after the loop exits.
    // read() will return EINTR since we register without SA_RESTART.
}

//from: https://github.com/kentonv/dvorak-qwerty/tree/master/unix
static int modifier_bit(int key) {
    switch (key) {
        case KEY_LEFTCTRL:
            return 1;
        case KEY_RIGHTCTRL:
            return 2;
        case KEY_LEFTALT:
            return 4;
        case KEY_LEFTMETA:
            return 8;
        default:
            return 0;
    }
}

// Maps a physical QWERTY key to the keycode that produces the correct
// Dvorak character when the OS layout is set to QWERTY.
// e.g. physical 's' -> KEY_O because 's' position in Dvorak produces 'o'.
static int dvorak_remap(int key) {
    switch (key) {
        case KEY_MINUS:      return KEY_LEFTBRACE;
        case KEY_EQUAL:      return KEY_RIGHTBRACE;
        case KEY_Q:          return KEY_APOSTROPHE;
        case KEY_W:          return KEY_COMMA;
        case KEY_E:          return KEY_DOT;
        case KEY_R:          return KEY_P;
        case KEY_T:          return KEY_Y;
        case KEY_Y:          return KEY_F;
        case KEY_U:          return KEY_G;
        case KEY_I:          return KEY_C;
        case KEY_O:          return KEY_R;
        case KEY_P:          return KEY_L;
        case KEY_LEFTBRACE:  return KEY_SLASH;
        case KEY_RIGHTBRACE: return KEY_EQUAL;
        case KEY_A:          return KEY_A;
        case KEY_S:          return KEY_O;
        case KEY_D:          return KEY_E;
        case KEY_F:          return KEY_U;
        case KEY_G:          return KEY_I;
        case KEY_H:          return KEY_D;
        case KEY_J:          return KEY_H;
        case KEY_K:          return KEY_T;
        case KEY_L:          return KEY_N;
        case KEY_SEMICOLON:  return KEY_S;
        case KEY_APOSTROPHE: return KEY_MINUS;
        case KEY_Z:          return KEY_SEMICOLON;
        case KEY_X:          return KEY_Q;
        case KEY_C:          return KEY_J;
        case KEY_V:          return KEY_K;
        case KEY_B:          return KEY_X;
        case KEY_N:          return KEY_B;
        case KEY_M:          return KEY_M;
        case KEY_COMMA:      return KEY_W;
        case KEY_DOT:        return KEY_V;
        case KEY_SLASH:      return KEY_Z;
        default:             return key;
    }
}
static ssize_t emit(int fd, int type, int code, int value, struct timeval time) {
    struct input_event ev = {0};
    ev.type = type;
    ev.code = code;
    ev.value = value;
    ev.time = time;
    //fprintf(stdout, "Emit event type=%d code=%d value=%d\n",ev.type, ev.code, ev.value);
    return write(fd, &ev, sizeof(ev));
}

static bool has_event_type(const unsigned int array_bit_ev[], int event_type) {
    return (array_bit_ev[event_type/32] & (1U << (event_type % 32))) != 0;
}

static bool setup_event_type(int out_fd, unsigned long event_type, int max_val, const unsigned int array_bit[]) {
    for (int i = 0; i < max_val; i++) {
        if (!(array_bit[i / 32] & (1U << (i % 32))))
            continue;

        switch(event_type) {
            case UI_SET_EVBIT:
                if (ioctl(out_fd, UI_SET_EVBIT, i) < 0) {
                    fprintf(stderr, "Cannot set EV bit %d: %s\n", i, strerror(errno));
                    return false;
                }
                break;
            case UI_SET_KEYBIT:
                if (ioctl(out_fd, UI_SET_KEYBIT, i) < 0) {
                    fprintf(stderr, "Cannot set KEY bit %d: %s\n", i, strerror(errno));
                    return false;
                }
                break;
            case UI_SET_RELBIT:
                if (ioctl(out_fd, UI_SET_RELBIT, i) < 0) {
                    fprintf(stderr, "Cannot set REL bit %d: %s\n", i, strerror(errno));
                    return false;
                }
                break;
            case UI_SET_ABSBIT: {
                struct uinput_abs_setup abs_setup = { .code = i };
                if (ioctl(fdi, EVIOCGABS(i), &abs_setup.absinfo) < 0) {
                    fprintf(stderr, "Failed to get ABS info for axis %d: %s\n", i, strerror(errno));
                    continue;
                }
                if (ioctl(out_fd, UI_ABS_SETUP, &abs_setup) < 0) {
                    fprintf(stderr, "Failed to setup ABS axis %d: %s\n", i, strerror(errno));
                    continue;
                }
                if (ioctl(out_fd, UI_SET_ABSBIT, i) < 0) {
                    fprintf(stderr, "Cannot set ABS bit %d: %s\n", i, strerror(errno));
                    return false;
                }
                break;
            }
            case UI_SET_MSCBIT:
                if (ioctl(out_fd, UI_SET_MSCBIT, i) < 0) {
                    fprintf(stderr, "Cannot set MSC bit %d: %s\n", i, strerror(errno));
                    return false;
                }
                break;
        }
    }
    return true;
}

// Tracks keys that were remapped on press, so release/repeat can emit
// the correct code regardless of modifier state at that time.
typedef struct { int original; int emitted; } KeyPair;

// Returns the emitted code for original, or -1 if not found.
static int remapped_find(const KeyPair *keys, int count, int original) {
    for (int i = 0; i < count; i++)
        if (keys[i].original == original) return keys[i].emitted;
    return -1;
}

// Removes the entry for original. Returns the emitted code, or -1 if not found.
static int remapped_remove(KeyPair *keys, int *count, int original) {
    for (int i = 0; i < *count; i++) {
        if (keys[i].original != original) continue;
        int emitted = keys[i].emitted;
        // Shift remaining entries down to keep array compact.
        (*count)--;
        for (int j = i; j < *count; j++)
            keys[j] = keys[j + 1];
        keys[*count] = (KeyPair){0, 0};
        return emitted;
    }
    return -1;
}

// Always-on key cycle: capslock→enter→backspace→escape→capslock.
// Applied before anything else, bypasses dvorak translation entirely.
static int custom_cycle(int key) {
    switch (key) {
        case KEY_CAPSLOCK:  return KEY_ENTER;
        case KEY_ENTER:     return KEY_BACKSPACE;
        case KEY_BACKSPACE: return KEY_ESC;
        case KEY_ESC:       return KEY_CAPSLOCK;
        default:            return key;
    }
}

// Swaps applied during normal typing (no modifier), before dvorak translation.
// Keys are identified by what they output in Dvorak:
//   ' "  <->  ; :   (KEY_Q position swaps with KEY_Z position)
//   / ?  <->  z     (KEY_LEFTBRACE position swaps with KEY_SLASH position)
static int custom_swap(int key) {
    switch (key) {
        case KEY_Q:          return KEY_Z;
        case KEY_Z:          return KEY_Q;
        case KEY_LEFTBRACE:  return KEY_SLASH;
        case KEY_SLASH:      return KEY_LEFTBRACE;
        default:             return key;
    }
}

#define FAIL_FDI(msg, ...) do { fprintf(stderr, msg, ##__VA_ARGS__); close(fdi); return EXIT_FAILURE; } while(0)

static void usage(const char *path) {
    /* take only the last portion of the path */
    const char *basename = strrchr(path, '/');
    basename = basename ? basename + 1 : path;

    fprintf(stderr, "usage: %s [OPTION]\n", basename);
    fprintf(stderr, "  -d /dev/input/by-id/…\t"
                    "Specifies which device should be captured.\n");
    fprintf(stderr, "  -m STRING\t\t"
                    "Match only the STRING with the USB device name. \n"
                    "\t\t\tSTRING can contain multiple words, separated by space.\n");
    fprintf(stderr, "example: %s -u -d /dev/input/by-id/usb-Logitech_USB_Receiver-if02-event-kbd -m \"k750 k350\"\n", basename);
}

int main(int argc, char *argv[]) {
    // Use sigaction without SA_RESTART so that the blocking read() in the
    // event loop returns EINTR when a signal arrives, allowing clean exit.
    struct sigaction sa = { .sa_handler = sig_handler, .sa_flags = 0 };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);

    int opt;
    char *device = NULL,
         *match = NULL;
    while ((opt = getopt(argc, argv, "d:m:tc")) != -1) {
        switch (opt) {
            case 'd':
                device = optarg;
                break;
            case 'm':
                match = optarg;
                break;
            default:
                usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    if (device == NULL) {
        usage(argv[0]);
        fprintf(stderr, "Error: Input device not specified.\n");
        fprintf(stderr, "Hint: Provide a valid input device, typically found under /dev/input/by-id/...\n");
        return EXIT_FAILURE;
    }

    //Start the fdi setup
    fdi = open(device, O_RDONLY);
    if (fdi < 0) {
        fprintf(stderr, "Error: Failed to open device [%s]: %s.\n", device, strerror(errno));
        fprintf(stderr, "Hint: Check if the device path is correct and you have the necessary permissions.\n");
        return EXIT_FAILURE;
    }

    char keyboard_name[UINPUT_MAX_NAME_SIZE] = "Unknown";
    if (ioctl(fdi, EVIOCGNAME(sizeof(keyboard_name) - 1), keyboard_name) < 0) {
        fprintf(stderr, "Error: Unable to retrieve device name for [%s]: %s.\n", device, strerror(errno));
        fprintf(stderr, "Hint: Verify if the device is functional and properly configured.\n");
        close(fdi);
        return EXIT_FAILURE;
    }

    struct uinput_setup usetup =
            { .id =
                { .bustype = BUS_USB, .vendor = 0x1111, .product = 0x2222 },
              .name = "Virtual Dvorak Keyboard" };
    if (strcmp(keyboard_name, usetup.name) == 0) {
        fprintf(stdout, "Info: Skipping mapping for the device we just created: %s.\n", keyboard_name);
        close(fdi);
        return EXIT_SUCCESS;
    }

    if (match != NULL) {
        bool matched = false;
        for (char *token = strtok(match, " "); token != NULL; token = strtok(NULL, " ")) {
            if (strcasestr(keyboard_name, token) == NULL) continue;
            fprintf(stdout, "Info: Found matching input: [%s] for device [%s].\n", keyboard_name, device);
            matched = true;
            break;
        }
        if (!matched) {
            fprintf(stderr, "Error: Device [%s] does not match any of the specified keywords: [%s].\n", keyboard_name, match);
            close(fdi);
            return EXIT_FAILURE;
        }
    }

    // Read capabilities
    unsigned int
        array_bit_ev[EV_MAX/32 + 1]= {0},
        array_bit_key[KEY_MAX/32 + 1]= {0},
        array_bit_rel[REL_MAX/32 + 1]= {0},
        array_bit_abs[ABS_MAX/32 + 1]= {0},
        array_bit_msc[MSC_MAX/32 + 1]= {0};

    if (ioctl(fdi, EVIOCGBIT(0, sizeof(array_bit_ev)), &array_bit_ev) < 0)
        FAIL_FDI("Error: Failed to retrieve event capabilities for device [%s]: %s.\n", device, strerror(errno));

    if (has_event_type(array_bit_ev, EV_KEY) && ioctl(fdi, EVIOCGBIT(EV_KEY, sizeof(array_bit_key)), &array_bit_key) < 0)
        FAIL_FDI("Error: Failed to retrieve EV_KEY capabilities for device [%s]: %s.\n", device, strerror(errno));

    if (has_event_type(array_bit_ev, EV_REL) && ioctl(fdi, EVIOCGBIT(EV_REL, sizeof(array_bit_rel)), &array_bit_rel) < 0)
        FAIL_FDI("Error: Failed to retrieve EV_REL capabilities for device [%s]: %s.\n", device, strerror(errno));

    if (has_event_type(array_bit_ev, EV_ABS) && ioctl(fdi, EVIOCGBIT(EV_ABS, sizeof(array_bit_abs)), &array_bit_abs) < 0)
        FAIL_FDI("Error: Failed to retrieve EV_ABS capabilities for device [%s]: %s.\n", device, strerror(errno));

    if (has_event_type(array_bit_ev, EV_MSC) && ioctl(fdi, EVIOCGBIT(EV_MSC, sizeof(array_bit_msc)), &array_bit_msc) < 0)
        FAIL_FDI("Error: Failed to retrieve EV_MSC capabilities for device [%s]: %s.\n", device, strerror(errno));

    //Check we are a keyboard
    if (!(array_bit_key[KEY_X / 32] & (1 << (KEY_X % 32))) ||
        !(array_bit_key[KEY_C / 32] & (1 << (KEY_C % 32))) ||
        !(array_bit_key[KEY_V / 32] & (1 << (KEY_V % 32)))) {
        fprintf(stdout, "Info: Device [%s] is not recognized as a keyboard (missing essential keys).\n", device);
        close(fdi);
        return EXIT_SUCCESS;
    }

    // Start the uinput setup.
    // O_NONBLOCK prevents open() from blocking during device initialisation,
    // but we clear it immediately so that writes block rather than silently
    // failing with EAGAIN under load.
    fdo = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fdo < 0) {
        fprintf(stderr, "Error: Failed to open /dev/uinput for device [%s]: %s.\n", device, strerror(errno));
        close(fdi);
        return EXIT_FAILURE;
    }
    fcntl(fdo, F_SETFL, fcntl(fdo, F_GETFL) & ~O_NONBLOCK);

    // Configure the virtual device
    if (ioctl(fdo, UI_DEV_SETUP, &usetup) < 0) {
        fprintf(stderr, "Error: Failed to configure the virtual device for [%s]: %s.\n", device, strerror(errno));
        close(fdo);
        close(fdi);
        return EXIT_FAILURE;
    }

    if(!setup_event_type(fdo, UI_SET_EVBIT, EV_SW, array_bit_ev)) {
        fprintf(stderr, "Cannot setup_event_type for UI_SET_EVBIT/device [%s]: %s.\n", device, strerror(errno));
        close(fdo);
        close(fdi);
        return EXIT_FAILURE;
    }

    if(!setup_event_type(fdo, UI_SET_KEYBIT, KEY_MAX, array_bit_key)) {
        fprintf(stderr, "Cannot setup_event_type for EV_KEY/device [%s]: %s.\n", device, strerror(errno));
        close(fdo);
        close(fdi);
        return EXIT_FAILURE;
    }

    if(!setup_event_type(fdo, UI_SET_RELBIT, REL_MAX, array_bit_rel)) {
        fprintf(stderr, "Cannot setup_event_type for EV_REL/device [%s]: %s.\n", device, strerror(errno));
        close(fdo);
        close(fdi);
        return EXIT_FAILURE;
    }

    if(!setup_event_type(fdo, UI_SET_ABSBIT, ABS_MAX, array_bit_abs)) {
        fprintf(stderr, "Cannot setup_event_type for EV_ABS/device [%s]: %s.\n", device, strerror(errno));
        close(fdo);
        close(fdi);
        return EXIT_FAILURE;
    }

    if(!setup_event_type(fdo, UI_SET_MSCBIT, MSC_MAX, array_bit_msc)) {
        fprintf(stderr, "Cannot setup_event_type for MSC_MAX/device [%s]: %s.\n", device, strerror(errno));
        close(fdo);
        close(fdi);
        return EXIT_FAILURE;
    }

    if (ioctl(fdo, UI_DEV_CREATE) < 0) {
        fprintf(stderr, "Cannot create device: %s.\n", strerror(errno));
        close(fdo);
        close(fdi);
        return EXIT_FAILURE;
    }

    // Wait for device to be ready
    usleep(200000);

    if (ioctl(fdi, EVIOCGRAB, 1) < 0) {
        fprintf(stderr, "Cannot grab key: %s.\n", strerror(errno));
        close(fdo);
        close(fdi);
        return EXIT_FAILURE;
    }

    struct input_event ev = {0};
    int mod_state = 0,
        remapped_count = 0;

    KeyPair remapped_keys[MAX_LENGTH] = {0};

    // Emergency exit: hold F9+F10+F11+F12 simultaneously to release the grab
    // and exit cleanly, leaving the keyboard in its normal passthrough state.
    #define EMERGENCY_F9  (1 << 0)
    #define EMERGENCY_F10 (1 << 1)
    #define EMERGENCY_F11 (1 << 2)
    #define EMERGENCY_F12 (1 << 3)
    #define EMERGENCY_ALL (EMERGENCY_F9 | EMERGENCY_F10 | EMERGENCY_F11 | EMERGENCY_F12)
    int emergency_state = 0;

    static const struct { int key; int bit; } emergency_keys[] = {
        { KEY_F9,  EMERGENCY_F9  },
        { KEY_F10, EMERGENCY_F10 },
        { KEY_F11, EMERGENCY_F11 },
        { KEY_F12, EMERGENCY_F12 },
    };

    fprintf(stderr, "Starting event loop with keyboard: [%s] for device [%s].\n", keyboard_name, device);

    while (keep_running) {
        ssize_t n = read(fdi, &ev, sizeof ev);
        if (n == (ssize_t) -1) {
            if (errno == EINTR) continue;
            break;
        } else if (n != sizeof ev) {
            break;
        }

        // Non-key events pass straight through.
        if (ev.type != EV_KEY) {
            emit(fdo, ev.type, ev.code, ev.value, ev.time);
            continue;
        }

        // Track emergency-exit key state and fire if all four are held.
        for (int i = 0; i < 4; i++) {
            if (ev.code != (unsigned)emergency_keys[i].key) continue;
            if (ev.value != 0) emergency_state |=  emergency_keys[i].bit;
            else               emergency_state &= ~emergency_keys[i].bit;
        }
        if (emergency_state == EMERGENCY_ALL) {
            fprintf(stderr, "Emergency exit triggered (F9+F10+F11+F12): releasing grab.\n");
            ioctl(fdi, EVIOCGRAB, 0);
            close(fdo);
            close(fdi);
            return EXIT_SUCCESS;
        }

        // Track modifier state.
        int mod_bit = modifier_bit(ev.code);
        if (mod_bit) {
            if (ev.value != 0) mod_state |=  mod_bit;  // press or repeat
            else               mod_state &= ~mod_bit;  // release
        }

        // Cycle remap (capslock/enter/backspace/escape) is always active and
        // must be checked before the repeat/release paths so that all three
        // event types (press, repeat, release) are translated consistently.
        // Without this, the OS sees mismatched down/up keycodes → stuck keys.
        int cycled = custom_cycle(ev.code);
        if (cycled != ev.code) {
            emit(fdo, ev.type, cycled, ev.value, ev.time);
            continue;
        }

        // Repeat and release are resolved from the tracking array using the
        // original keycode, before any translation.  This is critical: mod
        // state may have changed since the key was pressed, so recomputing
        // dvorak_code here would give the wrong answer and leak array entries.
        if (ev.value == 2) {
            int found = remapped_find(remapped_keys, remapped_count, ev.code);
            emit(fdo, ev.type, found >= 0 ? found : ev.code, ev.value, ev.time);
            continue;
        }
        if (ev.value == 0) {
            int found = remapped_remove(remapped_keys, &remapped_count, ev.code);
            emit(fdo, ev.type, found >= 0 ? found : ev.code, ev.value, ev.time);
            continue;
        }

        // From here on we only have key presses (value == 1).

        // With a modifier held, pass physical keycodes through so shortcuts
        // stay at their QWERTY positions.  Without a modifier, apply the
        // custom swaps first, then the full dvorak translation.
        int dvorak_code = (mod_state != 0)
            ? ev.code
            : dvorak_remap(custom_swap(ev.code));

        // Keys that translate to themselves need no tracking.
        if (dvorak_code == ev.code) {
            emit(fdo, ev.type, ev.code, ev.value, ev.time);
            continue;
        }

        // Remapped key press: track original→emitted so release can find it.
        if (remapped_count == MAX_LENGTH) {
            fprintf(stderr, "Warning: too many simultaneous remapped keys (%d), dropping 0x%04x.\n",
                    MAX_LENGTH, ev.code);
        } else {
            remapped_keys[remapped_count++] = (KeyPair){ ev.code, dvorak_code };
            emit(fdo, ev.type, dvorak_code, ev.value, ev.time);
        }
    }
    // Release any keys that were held when the loop exited, so the OS doesn't
    // see stuck keys after the virtual device is destroyed.
    // Each key-up must be followed by EV_SYN/SYN_REPORT — unlike normal
    // operation where SYN events come automatically from the physical device's
    // event stream, here we have to emit them manually.
    struct timeval now;
    gettimeofday(&now, NULL);
    for (int i = 0; i < remapped_count; i++) {
        if (remapped_keys[i].original == 0) continue;
        emit(fdo, EV_KEY, remapped_keys[i].emitted, 0, now);
        emit(fdo, EV_SYN, SYN_REPORT, 0, now);
    }
    if (fdi >= 0) ioctl(fdi, EVIOCGRAB, 0);
    if (fdi >= 0) close(fdi);
    if (fdo >= 0) close(fdo);
    return EXIT_SUCCESS;
}