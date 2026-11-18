"""All numbers and device tables of the USB diagnosis daemon."""

HTTP_PORT = 8001
HTTP_HOST = "0.0.0.0"
POLL_HINT_MS = 1000

LOG_DIR = "/home/pi/.rosmaster/diagnose/usb"
LOG_FILE_BYTES = 2_000_000
LOG_FILE_COUNT = 3
INCIDENT_FILES_KEEP = 200
SUMMARY_BOOTS_KEEP = 20
LOG_OWNER = "pi"
LOG_FILE_MODE = 0o644
LOG_DIR_MODE = 0o755
STORE_TAIL_BYTES = 65536  # enough lines to find the newest mono after a restart
BOOT_ID_FILE = "/proc/sys/kernel/random/boot_id"
MOCK_BOOT_ID = "mock"

TOPOLOGY_POLL_S = 1.0
SAMPLE_INTERVAL_S = 1.0
# ntp moves the wall clock after boot, smaller drift is uptime rounding
BOOT_WALL_STEP_S = 2.0
PROCESS_INTERVAL_S = 5.0
SUMMARY_INTERVAL_S = 30.0

INCIDENT_QUIET_S = 10.0
INCIDENT_MAX_S = 120.0
SNAPSHOT_BEFORE_S = 30.0
SAMPLE_RING_S = 900.0
PORT_EVENT_RING = 60
HISTORY_BUCKET_S = 300
HISTORY_BUCKETS = 12

MOTOR_WINDOW_S = 2.0
MOTOR_DELTA = 0.05  # m/s or rad/s change that counts as a start
BATTERY_DROP_V = 0.3
LOOP_REENUM_WINDOW_S = 3.0
LOOP_RESET_FAILS = 3
CPU_SPIKE_PCT = 80.0
CPU_WINDOW_S = 5.0
BOARD_SILENT_S = 2.0
RECENT_RECONNECT_S = 300.0

# judgement thresholds
EXT5V_GOOD_V = 4.95
EXT5V_LOW_V = 4.80
BATTERY_GOOD_V = 11.3
BATTERY_LOW_V = 10.5
TEMP_WARM_C = 70.0
TEMP_HOT_C = 80.0
CPU_WARN_PCT = 60.0  # below the spike threshold, above this only "hoch"

BOARD_ID = ("1a86", "7523")

# (vid, pid) -> (German name, role)
KNOWN_DEVICES: dict[tuple[str, str], tuple[str, str]] = {
    ("1a86", "7523"): ("Board-Mikrocontroller", "board"),
    ("10c4", "ea60"): ("Lidar", "lidar"),
    ("2bc5", "060f"): ("Tiefenkamera", "camera"),
    ("2bc5", "050f"): ("Farbkamera", "camera"),
    ("046d", "0825"): ("Webcam C270", "camera"),
    ("2109", "2817"): ("Expansion-Board-Hub", "hub"),
    ("2109", "0817"): ("Expansion-Board-Hub (USB3)", "hub"),
    ("05e3", "0608"): ("Kamera-Hub", "hub"),
}

# one entry per physical socket; lanes are root ports in device-path form
PI_SOCKETS: list[dict] = [
    {"id": "pi-usb3-a", "label": "USB3 oben", "kind": "usb3", "lanes": ["1-1", "2-1"], "slot": 0},
    {"id": "pi-usb3-b", "label": "USB3 unten", "kind": "usb3", "lanes": ["3-1", "4-1"], "slot": 1},
    {"id": "pi-usb2-a", "label": "USB2 oben", "kind": "usb2", "lanes": ["1-2"], "slot": 2},
    {"id": "pi-usb2-b", "label": "USB2 unten", "kind": "usb2", "lanes": ["3-2"], "slot": 3},
]
SOCKETS_VERIFIED = False

BUSY_PROCESSES = ("apt", "apt-get", "dpkg", "unattended-upgr", "dkms", "cc1", "cc1plus", "make")
SERVER_MARKER = "rosmaster_assist"

# board serial framing and usbmon reading
FRAME_MAX_EXT_LEN = 64  # longer length bytes are noise, not a frame
FRAME_BUFFER_MAX = 512
USBMON_ROOT = "/sys/kernel/debug/usb/usbmon"
USBMON_READ_BYTES = 65536
VCGENCMD_TIMEOUT_S = 2.0
THERMAL_FILE = "/sys/class/thermal/thermal_zone0/temp"
CPUFREQ_FILE = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"

# sysfs topology scan
SYSFS_USB_ROOT = "/sys/bus/usb/devices"
UNKNOWN_DEVICE_NAME = "Unbekanntes USB-Gerät"
NO_DEVICE_NAME = "nichts"
BOARD_MCU_SUFFIX = "Board-MCU"
MOTOR_TILE_LABEL = "Motorsteuerung"

# crash reason scores (higher = more likely shown first)
REASON_SCORES = {
    "motor_start": 0.9,
    "reenum_loop": 0.8,
    "heat": 0.7,
    "undervoltage": 0.85,
    "cpu_spike": 0.6,
    "board_mcu_silent": 0.9,
    "idle_unclear": 0.5,
}
MOTOR_BATTERY_BONUS = 0.1

# motor command scale and capture
MOTOR_SPEED_SCALE = 100  # raw MOTOR command units per 1.0 of full power
CAPTURE_IDLE_S = 5.0  # no URB from the board for this long means no traffic
CAPTURE_MAX_SPEED_MBPS = 5000  # a device this fast on the board bus disables capture

# page model
TRAFFIC_WINDOW_S = 5.0  # window for bytes/s and report rate
BATTERY_CURVE_POINTS = 60
INCIDENTS_ON_PAGE = 30
VERDICT_WINDOW_S = 3600.0
TILE_TECHNICAL_LINES = 20

# daemon loop
LOOP_SELECT_S = 0.25
USBMON_POLL_S = 0.2  # usbmon text files cannot be polled (select always says readable), so they are read on a timer
TOPOLOGY_DEBOUNCE_S = 0.2  # rescan this long after a kernel usb message
SOURCE_RETRY_S = 5.0  # a failed source is tried again after this
SOURCE_ERROR_REPEAT_S = 60.0  # the same error is logged at most once per this
SOURCE_ERROR_KEYS_MAX = 100
USBMON_PARTIAL_MAX = 8192  # longer unfinished usbmon lines are garbage
HTTP_POLL_S = 0.25
TICK_INTERVAL_S = 0.25  # state tick at most 4 Hz, however often usbmon traffic wakes the loop
STATE_ERROR_REPEAT_S = 60.0  # a failing state method is logged at most once per this

# mock mode
MOCK_LOG_DIR = "~/.rosmaster/diagnose/usb-mock"
MOCK_START_MONO = 160.0  # mock uptime at start, about a minute after the backlog burst
MOCK_FIRST_BURST_S = 20.0
MOCK_BURST_PERIOD_S = 90.0
MOCK_MOTOR_BURST_EVERY = 3
MOCK_MOTION_LEAD_S = 1.0
MOCK_MOTION_VX = 0.3
MOCK_MOTION_HOLD_S = 5.0
MOCK_RESTORE_S = (2.0, 4.0)
MOCK_LOOP_DELAY_S = 0.4  # orbbec over-current after its reconnect
MOCK_LOOP_RESTORE_S = 1.5
MOCK_URB_GAP_S = 0.05  # pause between two passes of the recorded usbmon lines
MOCK_CATCH_UP_S = 10.0  # skip recorded lines that are older than this
MOCK_EXT5V_V = (5.02, 5.07)
MOCK_TEMP_C = (55.0, 62.0)
MOCK_CPU_PCT = (10.0, 30.0)
MOCK_FREQ_MHZ = 2400.0
