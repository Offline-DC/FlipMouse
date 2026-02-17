/*
 * for enabling virtual mouse anywhere on the TCL FLIP 2
 * tyler boni <tyler.boni@gmail.com>
 *
 * Modified: add a control interface (files + UNIX socket) so other services/apps
 * can request enable/disable/toggle/status/quit.
 *
 * IMPORTANT BEHAVIOR CHANGE (per request):
 * - These controls are NOT "forced" overrides.
 * - Manual user toggle still works at all times.
 * - Control files are treated as one-shot commands (FlipMouse consumes/deletes them).
 */

// #define DEBUG 1

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libevdev/libevdev-uinput.h>
#include <dirent.h>
#include <stdarg.h>
#include <signal.h>
#include <sys/signalfd.h>

/* NEW: control interface */
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>

/* Configuration */
#define DEV_INPUT "/dev/input"
#define LOG_FILE "/cache/FlipMouse.log"
#ifdef DEBUG
#define ENABLE_LOG 1
#else
#define ENABLE_LOG 0
#endif

/* NEW: control paths (one-shot commands, not force) */
#define CONTROL_SOCK     "/data/local/tmp/flipmouse/sock"
#define STATUS_FILE      "/data/local/tmp/flipmouse/status"

/* Constants */
#define MIN_MOUSE_SPEED 1
#define WHEEL_SLOWDOWN_FACTOR 5
#define MAX_TOGGLE_HOLD_TIME 1

/* Event action return codes */
typedef enum
{
  CHANGED_TO_MOUSE = -2,
  MUTE_EVENT = 0,
  PASS_THRU_EVENT = 1,
  CHANGED_EVENT = 2
} event_action_t;

/* Keymap structure for mapping scancodes to keycodes */
typedef struct
{
  int scancode;
  int keycode;
} keymap_t;

/* Device structure */
typedef struct dev_st
{
  int fd;
  const char *name;
  struct libevdev *evdev;
  struct libevdev_uinput *uidev;
  struct dev_st *next;
} device_t;

/* Mouse configuration structure */
typedef struct
{
  int enabled;
  int toggle_down_at;
  int speed;
  int drag_mode;
  struct libevdev *dev;
  struct libevdev_uinput *uidev;
} mouse_t;

/* Global state */
typedef struct
{
  device_t *devices;
  mouse_t mouse;
  FILE *log_fp;
  const keymap_t *keymap;
  size_t keymap_size;
  volatile sig_atomic_t running;

  /* NEW: control interface state */
  int control_fd;
} app_state_t;

/* Device list */
static const char *supported_devices[] = {
    "mtk-kpd",
    "matrix-keypad",
    "AT Translated Set 2 keyboard", /* Laptop Keyboard */
    NULL};

/* Keymap configurations */
static const keymap_t keypad_keymap[] = {
    {35, KEY_UP},
    {9, KEY_DOWN},
    {19, KEY_LEFT},
    {34, KEY_RIGHT},
    {33, KEY_MENU}, /* scroll up */
    {2, KEY_SEND},  /* scroll down */
    {42, KEY_HELP}  /* Star key */
};

static const keymap_t laptop_keymap[] = {
    {200, KEY_UP},
    {208, KEY_DOWN},
    {203, KEY_LEFT},
    {205, KEY_RIGHT},
    {17, KEY_MENU},
    {31, KEY_SEND},
    {88, KEY_HELP} /* F12 key */
};

/* Global application state */
static app_state_t app_state = {0};

/* Function prototypes */
/* Mouse handling */
static int mouse_init(void);
static void mouse_cleanup(void);
static int mouse_toggle(struct input_event *ev);
static int mouse_handle_event(device_t *dev, struct input_event *ev);

/* Device handling */
static int devices_find_and_init(void);
static void devices_cleanup(void);

/* Event handling */
static int get_toggle_time(int event_time);
static int handle_input_event(device_t *dev, struct input_event *ev);
static int keymap_get_keycode(int scanvalue);
static int keymap_get_scanvalue(int keycode);

/* Logging */
static void log_init(void);
static void log_close(void);
static void log_message(const char *format, ...);
static void log_perror(const char *prefix);
static void log_event(const char *prefix, struct input_event *ev);

/* Signal handling */
static void signal_handler(int sig);
static void setup_signal_handlers(void);

/* Control interface */
static int control_init(void);
static void control_cleanup(void);
static void control_handle_ready(void);
static void write_status_file(void);

/* Main loop */
static int run_event_loop(void);

static int mouse_toggle(struct input_event *ev);

/* --- Logging Functions --- */

static void log_init(void)
{
  if (!ENABLE_LOG)
    return; /* Logging disabled */

  app_state.log_fp = fopen(LOG_FILE, "a");
  if (!app_state.log_fp)
  {
    perror("Failed to open log file");
    return; /* Continue without logging */
  }
  fprintf(app_state.log_fp, "\n----- FlipMouse Log initialized -----\n");
  fflush(app_state.log_fp);
}

static void log_close(void)
{
  if (app_state.log_fp)
  {
    fclose(app_state.log_fp);
    app_state.log_fp = NULL;
  }
}

static void log_perror(const char *prefix)
{
  if (!ENABLE_LOG)
    return;
  int err = errno; /* Save errno because log_message might change it */
  log_message("%s: %s (errno=%d)", prefix, strerror(err), err);
}

static void log_message(const char *format, ...)
{
  if (!ENABLE_LOG)
    return;

  va_list args;
  char log_buffer[256];

  va_start(args, format);
  vsnprintf(log_buffer, sizeof(log_buffer), format, args);
  va_end(args);

  if (app_state.log_fp)
  {
    fprintf(app_state.log_fp, "%s\n", log_buffer); /* Write to log file */
    fflush(app_state.log_fp);                      /* Ensure the log is written immediately */
  }

#ifdef DEBUG
  printf("%s\n", log_buffer); /* Print to console for debugging */
#endif
}

static void log_event(const char *prefix, struct input_event *ev)
{
  if (!ENABLE_LOG || ev->type == EV_SYN)
    return;

  char event_info[256];
  snprintf(event_info, sizeof(event_info),
           "%s [%s] Event: time %ld.%06ld, type %d (%s), code %d (%s), value %d",
           prefix,
           app_state.mouse.enabled ? "GRAB" : "PASS",
           ev->input_event_sec,
           ev->input_event_usec,
           ev->type,
           libevdev_event_type_get_name(ev->type),
           ev->code,
           libevdev_event_code_get_name(ev->type, ev->code),
           ev->value);

  log_message("%s", event_info);
}

/* --- Keymap Functions --- */

static int keymap_get_scanvalue(int keycode)
{
  for (size_t i = 0; i < app_state.keymap_size; i++)
  {
    if (app_state.keymap[i].keycode == keycode)
      return app_state.keymap[i].scancode;
  }
  return -1; /* Not found */
}

static int keymap_get_keycode(int scanvalue)
{
  for (size_t i = 0; i < app_state.keymap_size; i++)
  {
    if (app_state.keymap[i].scancode == scanvalue)
      return app_state.keymap[i].keycode;
  }
  return -1; /* Not found */
}

/* --- Control Interface (files + socket) --- */

static int file_exists(const char *path)
{
  return access(path, F_OK) == 0;
}

static void write_status_file(void) {
  int fd = open(STATUS_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd < 0) return;
  dprintf(fd, "enabled=%d speed=%d drag=%d\n",
          app_state.mouse.enabled, app_state.mouse.speed, app_state.mouse.drag_mode);
  close(fd);
}

static int control_init(void)
{
  app_state.control_fd = -1;

  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0)
  {
    log_perror("socket(AF_UNIX)");
    return -1;
  }

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, CONTROL_SOCK, sizeof(addr.sun_path) - 1);

  unlink(CONTROL_SOCK);

  if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    log_perror("bind(CONTROL_SOCK)");
    close(fd);
    return -1;
  }

  /* Allow other processes to talk to it (still depends on SELinux/root context) */
  chmod(CONTROL_SOCK, 0666);

  if (listen(fd, 4) < 0)
  {
    log_perror("listen(CONTROL_SOCK)");
    close(fd);
    unlink(CONTROL_SOCK);
    return -1;
  }

  app_state.control_fd = fd;
  log_message("Control socket listening at %s", CONTROL_SOCK);
  return 0;
}

static void control_cleanup(void)
{
  if (app_state.control_fd >= 0)
  {
    close(app_state.control_fd);
    app_state.control_fd = -1;
  }
  unlink(CONTROL_SOCK);
}

static void control_handle_command(int client_fd, const char *cmd)
{
  while (*cmd == ' ' || *cmd == '\n' || *cmd == '\r' || *cmd == '\t')
    cmd++;

  if (strncmp(cmd, "enable", 6) == 0)
  {
    app_state.mouse.enabled = 1;
    write_status_file();
    dprintf(client_fd, "ok enabled\n");
    log_message("Mouse enabled (socket)");
  }
  else if (strncmp(cmd, "disable", 7) == 0)
  {
    app_state.mouse.enabled = 0;
    write_status_file();
    dprintf(client_fd, "ok disabled\n");
    log_message("Mouse disabled (socket)");
  }
  else if (strncmp(cmd, "toggle", 6) == 0)
  {
    app_state.mouse.enabled = !app_state.mouse.enabled;
    dprintf(client_fd, "ok %s\n", app_state.mouse.enabled ? "enabled" : "disabled");
    log_message("Mouse toggled (socket) -> %s", app_state.mouse.enabled ? "enabled" : "disabled");
  }
  else if (strncmp(cmd, "status", 6) == 0)
  {
    dprintf(client_fd, "enabled=%d speed=%d drag=%d\n",
            app_state.mouse.enabled,
            app_state.mouse.speed,
            app_state.mouse.drag_mode);
  }
  else if (strncmp(cmd, "quit", 4) == 0)
  {
    dprintf(client_fd, "ok quitting\n");
    log_message("Quit requested (socket)");
    app_state.running = 0;
  }
  else
  {
    dprintf(client_fd, "err unknown_command\n");
  }
}

static void control_handle_ready(void)
{
  int cfd = accept(app_state.control_fd, NULL, NULL);
  if (cfd < 0)
    return;

  char buf[128];
  ssize_t n = read(cfd, buf, sizeof(buf) - 1);
  if (n > 0)
  {
    buf[n] = '\0';
    control_handle_command(cfd, buf);
  }

  close(cfd);
}

/* --- Mouse Functions --- */

static int mouse_init(void)
{
  app_state.mouse.dev = libevdev_new();
  log_message("Creating virtual mouse device");

  if (!app_state.mouse.dev)
  {
    log_message("ERROR: Failed to create virtual mouse device");
    return -1;
  }

  libevdev_set_name(app_state.mouse.dev, "FlipMouse Virtual Mouse");

  /* Configure mouse capabilities */
  libevdev_enable_event_code(app_state.mouse.dev, EV_REL, REL_X, NULL);
  libevdev_enable_event_code(app_state.mouse.dev, EV_REL, REL_Y, NULL);
  libevdev_enable_event_code(app_state.mouse.dev, EV_REL, REL_WHEEL, NULL);
  libevdev_enable_event_code(app_state.mouse.dev, EV_REL, REL_HWHEEL, NULL);
  libevdev_enable_event_code(app_state.mouse.dev, EV_KEY, BTN_LEFT, NULL);
  libevdev_enable_event_code(app_state.mouse.dev, EV_KEY, BTN_RIGHT, NULL);

  if (libevdev_uinput_create_from_device(app_state.mouse.dev,
                                         LIBEVDEV_UINPUT_OPEN_MANAGED,
                                         &app_state.mouse.uidev) < 0)
  {
    log_message("ERROR: Failed to create virtual mouse uinput device");
    libevdev_free(app_state.mouse.dev);
    app_state.mouse.dev = NULL;
    return -1;
  }

  /* Initialize mouse parameters */
  app_state.mouse.enabled = 0;
  app_state.mouse.speed = 4;
  app_state.mouse.drag_mode = 0;
  app_state.mouse.toggle_down_at = 0;

  log_message("Virtual mouse initialized successfully");
  return 0;
}

static void mouse_cleanup(void)
{
  if (app_state.mouse.uidev)
  {
    libevdev_uinput_destroy(app_state.mouse.uidev);
    app_state.mouse.uidev = NULL;
  }

  if (app_state.mouse.dev)
  {
    libevdev_free(app_state.mouse.dev);
    app_state.mouse.dev = NULL;
  }

  log_message("Virtual mouse resources released");
}

static int control_send_cmd(const char *cmd)
{
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) return 2;

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, CONTROL_SOCK, sizeof(addr.sun_path) - 1);

  if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(fd);
    return 3;
  }

  write(fd, cmd, strlen(cmd));
  write(fd, "\n", 1);

  char buf[256];
  ssize_t n = read(fd, buf, sizeof(buf) - 1);
  if (n > 0) {
    buf[n] = 0;
    write(STDOUT_FILENO, buf, n);
  }

  close(fd);
  return 0;
}

/* mouse_toggle(ev) */
static int mouse_toggle(struct input_event *ev)
{
  if (ev->value == 1)
  { /* Key press, not release */
    app_state.mouse.toggle_down_at = ev->input_event_sec;
    return CHANGED_TO_MOUSE;
  }
  else if (ev->value == 0 && app_state.mouse.toggle_down_at != 0)
  {
    /* if the key was pressed for less than time second, toggle mouse mode */
    if (get_toggle_time(ev->input_event_sec) < MAX_TOGGLE_HOLD_TIME)
    {
      app_state.mouse.enabled = !app_state.mouse.enabled;
      write_status_file();
      app_state.mouse.toggle_down_at = 0;
      log_message("Mouse mode %s (manual)", app_state.mouse.enabled ? "enabled" : "disabled");
      return CHANGED_TO_MOUSE;
    }
    return CHANGED_TO_MOUSE;
  }
  else
  {
    return MUTE_EVENT;
  }
}

static int get_toggle_time(int event_time)
{
  if (app_state.mouse.toggle_down_at == 0)
  {
    /* toggle key is not down */
    return 0;
  }

  int timediff = event_time - app_state.mouse.toggle_down_at;
  log_message("Toggle key held for %d seconds", timediff);
  return timediff;
}

static int mouse_handle_event(device_t *dev, struct input_event *ev)
{
  static unsigned int slowdown_counter = 0;
  int keycode = ev->code;

  /* Handle MSC_SCAN events for special keys */
  if (ev->type == EV_MSC)
  {
    if (keycode == MSC_SCAN)
    {
      keycode = keymap_get_keycode(ev->value);
      if (keycode != -1)
      {
        log_message("Scan code %d mapped to keycode %d", ev->value, keycode);
      }
    }
  }
  else if (ev->type == EV_KEY)
  {
    /* Skip KEY events that are handled via MSC_SCAN */
    if (keymap_get_scanvalue(keycode) != -1)
    {
      log_message("Keycode %d handled by MSC_SCAN", keycode);
      return MUTE_EVENT;
    }
  }

  /* Process according to keycode */
  switch (keycode)
  {
  case KEY_VOLUMEUP:
    if (ev->value == 1)
    { /* Key press */
      app_state.mouse.speed++;
      log_message("Mouse speed increased to %d", app_state.mouse.speed);
    }
    return MUTE_EVENT;

  case KEY_VOLUMEDOWN:
    if (ev->value == 1)
    { /* Key press */
      app_state.mouse.speed--;
      if (app_state.mouse.speed < MIN_MOUSE_SPEED)
        app_state.mouse.speed = MIN_MOUSE_SPEED;
      log_message("Mouse speed decreased to %d", app_state.mouse.speed);
    }
    return MUTE_EVENT;

  case KEY_ENTER:
    log_message("Mouse left click");
    ev->type = EV_KEY;
    ev->code = BTN_LEFT;
    return CHANGED_TO_MOUSE;

  case KEY_B:
    if (ev->value == 1)
    { /* Key press */
      app_state.mouse.drag_mode = !app_state.mouse.drag_mode;

      log_message("Drag mode %s", app_state.mouse.drag_mode ? "enabled" : "disabled");
      ev->type = EV_KEY;
      ev->code = BTN_LEFT;
      ev->value = app_state.mouse.drag_mode ? 1 : 0; /* 1=press, 0=release */

      return CHANGED_TO_MOUSE;
    }
    break;

  case KEY_UP:
    ev->type = EV_REL;
    ev->code = REL_Y;
    ev->value = -app_state.mouse.speed;
    return CHANGED_TO_MOUSE;

  case KEY_DOWN:
    ev->type = EV_REL;
    ev->code = REL_Y;
    ev->value = app_state.mouse.speed;
    return CHANGED_TO_MOUSE;

  case KEY_LEFT:
    ev->type = EV_REL;
    ev->code = REL_X;
    ev->value = -app_state.mouse.speed;
    return CHANGED_TO_MOUSE;

  case KEY_RIGHT:
    ev->type = EV_REL;
    ev->code = REL_X;
    ev->value = app_state.mouse.speed;
    return CHANGED_TO_MOUSE;

  case KEY_MENU: /* Scroll up */
    if (slowdown_counter++ % WHEEL_SLOWDOWN_FACTOR)
      return MUTE_EVENT;

    ev->type = EV_REL;
    ev->code = REL_WHEEL;
    ev->value = 1;
    return CHANGED_TO_MOUSE;

  case KEY_SEND: /* Scroll down */
    if (slowdown_counter++ % WHEEL_SLOWDOWN_FACTOR)
      return MUTE_EVENT;

    ev->type = EV_REL;
    ev->code = REL_WHEEL;
    ev->value = -1;
    return CHANGED_TO_MOUSE;

  default:
    return PASS_THRU_EVENT;
  }

  return PASS_THRU_EVENT;
}

/* --- Device Management Functions --- */

static int devices_find_and_init(void)
{
  DIR *dir;
  struct dirent *file;
  int result = -1;

  dir = opendir(DEV_INPUT);
  if (!dir)
  {
    log_message("ERROR: Failed to open directory %s", DEV_INPUT);
    log_perror("opendir");
    return -1;
  }

  while ((file = readdir(dir)) != NULL)
  {
    char file_path[256];
    int found = 0;

    /* Skip non-character devices */
    if (file->d_type != DT_CHR)
      continue;

    snprintf(file_path, sizeof(file_path), "%s/%s", DEV_INPUT, file->d_name);
    log_message("Checking device %s", file_path);

    int fd = open(file_path, O_RDONLY);
    if (fd < 0)
    {
      log_message("ERROR: Failed to open device file %s", file_path);
      log_perror("open");
      continue;
    }

    struct libevdev *evdev = NULL;
    if (libevdev_new_from_fd(fd, &evdev) < 0)
    {
      log_message("ERROR: Failed to create libevdev from fd %d", fd);
      close(fd);
      continue;
    }

    /* Check if this is a device we're interested in */
    for (int i = 0; supported_devices[i]; i++)
    {
      if (strcmp(libevdev_get_name(evdev), supported_devices[i]) == 0)
      {
        log_message("Found supported device: %s", libevdev_get_name(evdev));

        /* Create device structure */
        device_t *dev = malloc(sizeof(device_t));
        if (!dev)
        {
          log_message("ERROR: Failed to allocate memory for device");
          libevdev_free(evdev);
          close(fd);
          continue;
        }

        /* Initialize device */
        dev->fd = fd;
        dev->name = libevdev_get_name(evdev);
        dev->evdev = evdev;
        dev->next = NULL;

        /* Try to grab device exclusively */
        if (ioctl(dev->fd, EVIOCGRAB, 1) < 0)
        {
          log_message("WARNING: Failed to grab device exclusively");
        }

        /* Create uinput device */
        if (libevdev_uinput_create_from_device(dev->evdev,
                                               LIBEVDEV_UINPUT_OPEN_MANAGED,
                                               &(dev->uidev)) < 0)
        {
          log_message("ERROR: Failed to create uinput device");
          libevdev_free(evdev);
          close(fd);
          free(dev);
          continue;
        }

        log_message("Successfully attached device: %s", dev->name);

        /* Set keymap based on device */
        if (i > 1)
        { /* For laptop keyboard */
          log_message("Using laptop keymap");
          app_state.keymap = laptop_keymap;
          app_state.keymap_size = sizeof(laptop_keymap) / sizeof(laptop_keymap[0]);
        }
        else
        { /* For TCL Flip2 keypad */
          app_state.keymap = keypad_keymap;
          app_state.keymap_size = sizeof(keypad_keymap) / sizeof(keypad_keymap[0]);
        }

        /* Add to device list */
        if (!app_state.devices)
        {
          app_state.devices = dev;
        }
        else
        {
          device_t *d = app_state.devices;
          while (d->next)
            d = d->next;
          d->next = dev;
        }

        found = 1;
        result = 0; /* Success */
        break;
      }
    }

    /* If not a supported device, cleanup resources */
    if (!found)
    {
      log_message("Device %s not in supported list", file_path);
      libevdev_free(evdev);
      close(fd);
    }
  }

  closedir(dir);
  return result;
}

static void devices_cleanup(void)
{
  device_t *curr = app_state.devices;

  while (curr)
  {
    device_t *next = curr->next;

    if (curr->uidev)
      libevdev_uinput_destroy(curr->uidev);

    if (curr->evdev)
      libevdev_free(curr->evdev);

    if (curr->fd >= 0)
      close(curr->fd);

    free(curr);
    curr = next;
  }

  app_state.devices = NULL;
  log_message("All input devices released");
}

/* --- Event Handling Functions --- */
static int handle_input_event(device_t *dev, struct input_event *ev)
{
  if (ev->type == MSC_SCAN && ev->code == MSC_SCAN)
  {
    int keycode = keymap_get_keycode(ev->value);
    if (keycode == KEY_HELP || keycode == KEY_F12)
    {
      int td = app_state.mouse.toggle_down_at;
      if (get_toggle_time(ev->input_event_sec) > MAX_TOGGLE_HOLD_TIME)
      {
        if (td > 1)
        {
          app_state.mouse.toggle_down_at = 1;
        }
        else if (td == 1)
        {
          app_state.mouse.toggle_down_at = 0;
        }
        else
        {
          return MUTE_EVENT;
        }
        /* Send Mock event */
        log_message("Trigger default button");
        ev->type = EV_KEY;
        ev->code = keycode;
        ev->value = app_state.mouse.toggle_down_at;
        return CHANGED_EVENT;
      }
    }
  }

  /* Check if it's the toggle key (manual always allowed) */
  if (ev->type == EV_KEY)
  {
    if (ev->code == KEY_HELP || ev->code == KEY_F12)
      return mouse_toggle(ev);
  }

  /* If not in mouse mode, just pass through */
  if (!app_state.mouse.enabled)
    return PASS_THRU_EVENT;

  /* Handle mouse events */
  return mouse_handle_event(dev, ev);
}

/* --- Signal Handling --- */

static void signal_handler(int sig)
{
  log_message("Received signal %d, shutting down", sig);
  app_state.running = 0;
}

static void setup_signal_handlers(void)
{
  /* Use simple signal() function to avoid struct sigaction issues */
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGHUP, signal_handler);
}

/* --- Main Event Loop --- */

static int run_event_loop(void)
{
  struct input_event event;
  int event_result;
  fd_set fds, rfds;
  int maxfd = 0;
  char prefix[8];

  /* Initialize file descriptor set */
  FD_ZERO(&fds);
  for (device_t *d = app_state.devices; d; d = d->next)
  {
    FD_SET(d->fd, &fds);
    if (d->fd >= maxfd)
      maxfd = d->fd + 1;
  }

  if (app_state.control_fd >= 0)
  {
    FD_SET(app_state.control_fd, &fds);
    if (app_state.control_fd >= maxfd)
      maxfd = app_state.control_fd + 1;
  }

  log_message("Entering main event loop");
  app_state.running = 1;

  while (app_state.running)
  {
    rfds = fds;

    /* We use a short timeout so command files can be noticed even if no input events happen. */
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000; /* 200ms */

    int sel = select(maxfd, &rfds, NULL, NULL, &tv);
    if (sel < 0)
    {
      if (errno == EINTR)
        continue; /* Interrupted by signal */

      log_message("ERROR: select() failed");
      log_perror("select");
      break;
    }

    /* Control socket ready? */
    if (sel > 0 && app_state.control_fd >= 0 && FD_ISSET(app_state.control_fd, &rfds))
    {
      control_handle_ready();
      /* continue to input handling; there may be both */
    }

    if (sel == 0)
    {
      /* timeout only; loop continues */
      continue;
    }

    for (device_t *d = app_state.devices; d; d = d->next)
    {
      if (!FD_ISSET(d->fd, &rfds))
        continue;

      if (read(d->fd, &event, sizeof(event)) != sizeof(event))
      {
        log_message("ERROR: Failed to read event");
        continue;
      }

#ifdef DEBUG
      snprintf(prefix, sizeof(prefix), "<%d<", d->fd);
      log_event(prefix, &event);
#endif

      /* Process event */
      event_result = handle_input_event(d, &event);

      /* Handle event based on result code */
      if (event_result > 0)
      {
        /* Forward to original device */
#ifdef DEBUG
        snprintf(prefix, sizeof(prefix), ">%d>", d->fd);
        log_event(prefix, &event);
#endif
        libevdev_uinput_write_event(d->uidev, event.type, event.code, event.value);
        libevdev_uinput_write_event(d->uidev, EV_SYN, SYN_REPORT, 0);
      }
      else if (event_result < 0)
      {
        /* Forward to virtual mouse */
#ifdef DEBUG
        log_event(">M>", &event);
#endif
        libevdev_uinput_write_event(app_state.mouse.uidev, event.type, event.code, event.value);
        libevdev_uinput_write_event(app_state.mouse.uidev, EV_SYN, SYN_REPORT, 0);
      }
      /* event_result == 0 means mute the event */
    }
  }

  return 0;
}

/* --- Main Function --- */

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  if (argc >= 2) {
  if (!strcmp(argv[1], "enable") ||
      !strcmp(argv[1], "disable") ||
      !strcmp(argv[1], "toggle") ||
      !strcmp(argv[1], "status") ||
      !strcmp(argv[1], "quit")) {
    return control_send_cmd(argv[1]);
  }
}

  /* Initialize logging */
  log_init();
  log_message("FlipMouse starting up");

  /* Set up signal handlers for clean shutdown */
  setup_signal_handlers();

  /* Find and initialize input devices */
  if (devices_find_and_init() != 0)
  {
    log_message("ERROR: Failed to find any supported input devices");
    log_close();
    return 1;
  }

  /* Initialize virtual mouse */
  if (mouse_init() != 0)
  {
    log_message("ERROR: Failed to initialize virtual mouse");
    devices_cleanup();
    log_close();
    return 1;
  }

  /* Initialize control interface (optional) */
  if (control_init() != 0)
  {
    log_message("WARNING: control interface failed to init (continuing)");
  }

  /* Run the main event loop */
  int result = run_event_loop();

  /* Clean up all resources */
  control_cleanup();
  mouse_cleanup();
  devices_cleanup();

  log_message("FlipMouse shutting down");
  log_close();

  return result;
}
