"""Mirror of components/bsp/include/bsp_params.h (firmware = ground truth) plus
the Pi-side wire protocol and fuzzy-supervisor tuning. Keep in sync with the ESP
build — a drift here silently corrupts kinematics or the byte layout.
"""

# ---- Drivetrain geometry (mirror bsp_params.h) -----------------------------
GEARBOX_RATIO      = 30.0
ENC_PPR_MOTOR      = 11
ENC_DECODE_MULT    = 4
ENC_COUNTS_PER_REV = ENC_PPR_MOTOR * ENC_DECODE_MULT * 30   # 1320
WHEEL_DIAMETER_M   = 0.065
WHEEL_RADIUS_M     = WHEEL_DIAMETER_M * 0.5                 # 0.0325
WHEEL_BASE_M       = 0.290    # measured 06/2026 (NOT the stale 0.180 in the doc)

# ---- Loop timing -----------------------------------------------------------
CTRL_LOOP_HZ  = 200    # ESP PID loop
COMM_LOOP_HZ  = 100    # Pi supervisor cadence (this project)
TELEM_LOOP_HZ = 50     # ESP -> Pi telemetry rate

# ---- Safety ----------------------------------------------------------------
CMD_TIMEOUT_US = 200_000           # ESP stops motors if cmd older than this
CMD_TIMEOUT_S  = CMD_TIMEOUT_US / 1e6

# ---- Power -----------------------------------------------------------------
VBAT_CELLS   = 3
VBAT_NOMINAL = 11.1

# ---- Serial link -----------------------------------------------------------
PI_UART_PORT = "/dev/serial0"
PI_UART_BAUD = 921600
# COBS eliminates 0x00 from the body, so 0x00 is the ONLY safe delimiter
# (0x7E would be unsafe — COBS output can contain it). Must match protocol.h.
PROTO_DELIM  = 0x00

# Packed little-endian struct formats — must match protocol.h byte-for-byte.
#   cmd_packet_t  : float omega_ref_l, omega_ref_r; uint8 mode; uint16 seq   (11 B)
#   telem_packet_t: float omega_meas_l/r, yaw, yaw_rate, pwm_l/r, vbat;
#                   uint16 fault_flags; uint16 seq                            (32 B)
CMD_FMT   = "<ffBH"
TELEM_FMT = "<7fHH"

# cmd_packet.mode — firmware FSM meaning (NOT a fuzzy on/off toggle).
MODE_IDLE  = 0
MODE_RUN   = 1
MODE_ESTOP = 2

# fault_flags bits (mirror services/include/safety.h fault_bit_t).
FAULT_NONE        = 0
FAULT_CMD_TIMEOUT = 1 << 0
FAULT_IMU_STALE   = 1 << 1
FAULT_ENC_INVALID = 1 << 2
FAULT_VBAT_LOW    = 1 << 3
FAULT_ESTOP       = 1 << 4
_FAULT_NAMES = {
    FAULT_CMD_TIMEOUT: "CMD_TIMEOUT",
    FAULT_IMU_STALE:   "IMU_STALE",
    FAULT_ENC_INVALID: "ENC_INVALID",
    FAULT_VBAT_LOW:    "VBAT_LOW",
    FAULT_ESTOP:       "ESTOP",
}


def fault_names(flags):
    """Decode a fault_flags bitfield into a list of names ([] if clean)."""
    return [name for bit, name in _FAULT_NAMES.items() if flags & bit]


# ---- Fuzzy supervisor tuning (Phase 8 initial design; tune in Phase 9) -----
# Block 1 — yaw stability (eψ, r_err -> Δω_yaw)
YAW_EPSI_RANGE = 30.0    # heading-error universe, ±deg
YAW_RERR_RANGE = 120.0   # yaw-rate-error universe, ±deg/s
YAW_DW_MAX     = 3.0     # |Δω_yaw| output saturation, rad/s

# Block 2 — traction (σ_err, |r_err| -> λ)
TRAC_SIGMA_MAX = 1.0     # slip-error universe, [0,1]
TRAC_ABSR_MAX  = 120.0   # |yaw-rate error| universe, [0,120] deg/s
LAMBDA_MIN     = 0.4     # most we ever cut traction setpoints
LAMBDA_MAX     = 1.0     # full commanded speed (no slip)

# Slip proxy (services/expected.py): the wheel-vs-IMU yaw-rate residual at which
# σ saturates to 1, and how much of an intentional turn is treated as "expected"
# (subtracted so a clean commanded curve is not flagged as slip).
SLIP_SCALE_DPS   = 60.0
SLIP_EXPECT_GAIN = 0.0   # σ_expected = gain * |w_cmd|(deg/s); 0 until tuned
