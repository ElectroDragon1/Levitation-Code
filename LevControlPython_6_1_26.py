#!/usr/bin/env python3
"""
Maglev Controller — two modes:
  1. PID closed-loop control (Baumer → calibrated distance → PID → Nucleo PWM)
  2. Manual PWM injection (for characterizing PWM ↔ magnet current)
"""

import serial
import serial.tools.list_ports
import time
import numpy as np
import threading
from collections import deque  
import ctypes
import sys
import signal
import msvcrt


ctypes.windll.winmm.timeBeginPeriod(1)   # Windows: set system timer to 1 ms resolution


# ============ Configuration ============
BAUMER_PORT = 'COM10'
BAUMER_BAUD = 115200
NUCLEO_BAUD = 115200
PWM_MAX = 4095   
# === Maglev physics — z is in METERS in F = K·(I/z)² ===
MASS_KG    = 8.6          # measured with sensor and mounts attached
G          = 9.81

# === K(z) lookup table — N·m²/A² (z in mm) ===
# Measured statically with the scale:
#   F_lift = (MASS_KG - scale_kg) · g
#   K      = F_lift · (z/1000)² / I²
# Add rows as more data points become available. k_at() clamps to LUT
# range — don't extrapolate the magnetic model past where we've measured.
K_LUT_Z_MM = np.array([
      3.96,
      4.92,
      5.93,
      6.95,
      7.99,
      8.98,
     9.97,
     11,
  ], dtype=float)
K_LUT_VALUES = np.array([
    0.000488,
    0.000465,
    0.000442,
    0.000454,
    0.000424,
    0.000433,
    0.000437,
    0.000429,
  ], dtype=float)


def k_at(z_mm: float) -> float:
    """K(z) interpolated from K_LUT. Clamps to the measured range."""
    z = max(K_LUT_Z_MM[0], min(K_LUT_Z_MM[-1], float(z_mm)))
    return float(np.interp(z, K_LUT_Z_MM, K_LUT_VALUES))

# === LAST KNOWN GOOD VALUES ===

#SETPOINT_MM      = 7.0        # 5A limit forces us below ~9.3 mm for 8.5 kg
#MAX_USEFUL_GAP   = 15        # above this, 5A can't hold the magnet — turn off


#FF_TRIM          = 1.4      # keep
#KP_POS           = 0.1        # A/mm — slightly less aggressive
#KI_POS           = 0.001       # A/(mm·s) — less integrator authority needed with anti-windup
#KD_POS           = 0.012       # A·s/mm — more damping to kill velocity at plate approaches
#DERIV_FILTER_TAU = 0.001      # keep
#INTEGRAL_LIMIT   = 10.0       # mm·s — Ki×LIMIT = 0.5 A integrator authority
#LOOP_HZ          = 1000


# === LAST KNOWN GOOD VALUES ===
SETPOINT_MM      = 7        # 5A limit forces us below ~9.3 mm for 8.5 kg
MAX_USEFUL_GAP   = 15        # above this, 5A can't hold the magnet — turn off


FF_TRIM          = 1.40      # keep
KP_POS           = 0.13      # A/mm — slightly less aggressive
KI_POS           = 0.001       # A/(mm·s) — less integrator authority needed with anti-windup
KD_POS           = 0.013       # A·s/mm — more damping to kill velocity at plate approaches
DERIV_FILTER_TAU = 0.001      # keep
INTEGRAL_LIMIT   = 10.0       # mm·s — Ki×LIMIT = 0.5 A integrator authority
LOOP_HZ          = 1000










CAL_PWM = np.array([
    1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900,
    2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900,
    3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800,
], dtype=float)

# Wall current (drawn from supply) — used by Mode 1 PID and Mode 3
CAL_CURRENT = np.array([
    0.016, 0.024, 0.041, 0.064, 0.095, 0.131, 0.175, 0.224, 0.279, 0.340,
    0.408, 0.480, 0.559, 0.644, 0.737, 0.836, 0.943, 1.057, 1.181, 1.313,
    1.456, 1.616, 1.795, 1.993, 2.224, 2.505, 2.871, 3.735, 3.863,
], dtype=float)
MAX_CURRENT_A = 6.0  

# Coil current (through magnet, measured by INA260) — used by Mode 4 and Mode 1 display
CAL_COIL_CURRENT = np.array([
    0.069, 0.162, 0.277, 0.398, 0.520, 0.630, 0.749, 0.876, 0.991, 1.096,
    1.211, 1.326, 1.442, 1.556, 1.668, 1.783, 1.896, 2.010, 2.116, 2.229,
    2.355, 2.476, 2.607, 2.746, 2.910, 3.101, 3.305, 3.778, 3.851,
], dtype=float)
MAX_COIL_CURRENT_A = 6.0


def coil_current_local_slope(pwm_value: int) -> float:
    """Local dI_coil/dPWM (A per PWM count) at a given PWM. From LUT."""
    pwm_value = max(int(CAL_PWM[0]), min(int(CAL_PWM[-1]), int(pwm_value)))
    idx = int(np.searchsorted(CAL_PWM, pwm_value))
    if idx == 0:
        idx = 1
    elif idx >= len(CAL_PWM):
        idx = len(CAL_PWM) - 1
    return (CAL_COIL_CURRENT[idx] - CAL_COIL_CURRENT[idx-1]) / (CAL_PWM[idx] - CAL_PWM[idx-1])

# Baumer protocol
POLL_CMD = bytes.fromhex('010103100200f0ff')
RAW_TO_MM = 0.001
CAL_A = 0.002482344
CAL_B = 0.503972
CAL_C = 0.154841

# ============ Current Sensor Stuff============
# ============ Current Sensor Stuff ============

class CurrentReader:
    def __init__(self, ser, history_size=32):
        self.ser = ser
        self.latest_a = None
        self.latest_t = 0.0
        self._history = deque(maxlen=history_size)
        self._buf = b''
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                n = self.ser.in_waiting
            except Exception:
                time.sleep(0.05)
                continue
            if n == 0:
                time.sleep(0.005)
                continue
            try:
                self._buf += self.ser.read(n)
            except Exception:
                time.sleep(0.05)
                continue
            while b'\n' in self._buf:
                line, self._buf = self._buf.split(b'\n', 1)
                try:
                    s = line.decode('ascii', errors='ignore').strip()
                except Exception:
                    continue
                if s.startswith('I:'):
                    try:
                        val = float(s[2:])  # signed: negative means reverse drive
                        self.latest_a = val
                        self.latest_t = time.time()
                        self._history.append(val)
                    except ValueError:
                        pass

    def average(self, n=None):
        if not self._history:
            return None
        samples = list(self._history) if n is None else list(self._history)[-n:]
        return sum(samples) / len(samples)

    def stats(self, n=None):
        if not self._history:
            return None, None
        samples = list(self._history) if n is None else list(self._history)[-n:]
        m = sum(samples) / len(samples)
        var = sum((x - m) ** 2 for x in samples) / len(samples)
        return m, var ** 0.5

    def stop(self):
        self._stop.set()

# ============ Baumer ============

def calibrate(meas_mm):
    return CAL_A * meas_mm * meas_mm + CAL_B * meas_mm + CAL_C

def coil_current_to_pwm(amps: float) -> int:
    """Convert desired coil current (A) → PWM via coil LUT."""
    if amps <= 0:
        return 0
    if amps >= MAX_COIL_CURRENT_A:
        amps = MAX_COIL_CURRENT_A
    pwm = float(np.interp(amps, CAL_COIL_CURRENT, CAL_PWM))
    pwm = int(round(pwm))
    if pwm < 0:
        pwm = 0
    elif pwm > PWM_MAX:
        pwm = PWM_MAX
    return pwm

def pwm_to_coil_current(pwm: int) -> float:
    """Convert PWM → expected coil current (A) via coil LUT."""
    pwm = max(0, min(int(pwm), PWM_MAX))
    if pwm < CAL_PWM[0]:
        return 0.0   # below dead zone
    return float(np.interp(pwm, CAL_PWM, CAL_COIL_CURRENT))

def capture_step_response(nucleo, reader, target_amps,
                          settle_time=0.3, capture_duration=0.5):
    """Send target_amps as a current command, log samples, return analysis dict."""
    time.sleep(settle_time)
    baseline = reader.average(n=10)
    if baseline is None:
        baseline = 0.0

    last_t = reader.latest_t
    samples = []

    t_send = time.time()
    nucleo = send_current(nucleo, target_amps)   # ← was: send_pwm(nucleo, target_pwm)

    while time.time() - t_send < capture_duration:
        if reader.latest_t != last_t:
            last_t = reader.latest_t
            samples.append((last_t - t_send, reader.latest_a))
        time.sleep(0.002)

    if len(samples) < 3:
        return nucleo, {'baseline': baseline, 'final': None,
                        'latency': None, 'rise_time': None,
                        'samples': samples}

    final = float(np.mean([c for _, c in samples[-5:]]))
    step = final - baseline

    latency = None
    rise_time = None
    if abs(step) > 0.005:
        # latency: first crossing of baseline + 5% of step
        thresh_detect = baseline + 0.05 * step
        thresh10 = baseline + 0.10 * step
        thresh90 = baseline + 0.90 * step
        rising = step > 0

        def crossed(c, thresh):
            return c >= thresh if rising else c <= thresh

        for t, c in samples:
            if latency is None and crossed(c, thresh_detect):
                latency = t
                break

        t10 = next((t for t, c in samples if crossed(c, thresh10)), None)
        t90 = next((t for t, c in samples if crossed(c, thresh90)), None)
        if t10 is not None and t90 is not None and t90 >= t10:
            rise_time = t90 - t10

    return nucleo, {'baseline': baseline, 'final': final,
                    'latency': latency, 'rise_time': rise_time,
                    'samples': samples}


def show_live_readout(reader, duration=3.0, baumer=None):
    """Live readout. Shows current; if `baumer` is provided, shows distance too.
    `duration=None` runs until Ctrl+C."""
    if duration is None:
        print('  Live readout (Ctrl+C to return to prompt):')
    else:
        print(f'  Live readout ({duration:.1f}s, Ctrl+C to skip):')
    end_time = None if duration is None else time.time() + duration
    try:
        while end_time is None or time.time() < end_time:
            m = reader.latest_a
            i_str = f'{m:.3f} A' if m is not None else '   ?   '
            if baumer is not None:
                result = read_distance(baumer)
                z_str = f'{result[2]:6.2f} mm' if result is not None else '   ?   '
                print(f'\r    I={i_str:>9}   z={z_str:>10}    ',
                      end='', flush=True)
            else:
                print(f'\r    {i_str:>9}    ', end='', flush=True)
            time.sleep(0.05)
        print()
    except KeyboardInterrupt:
        print()


def current_to_pwm(amps: float) -> int:
    """Convert desired coil current (A) → PWM (0..PWM_MAX) via measured LUT."""
    if amps <= 0:
        return 0
    if amps >= MAX_CURRENT_A:
        amps = MAX_CURRENT_A
    # np.interp handles the math: monotonic increasing CAL_CURRENT means
    # we get the corresponding PWM with linear interpolation between rows.
    pwm = float(np.interp(amps, CAL_CURRENT, CAL_PWM))
    pwm = int(round(pwm))
    if pwm < 0:
        pwm = 0
    elif pwm > PWM_MAX:
        pwm = PWM_MAX
    return pwm

def parse_baumer_response(data):
    if len(data) < 17:
        return None
    if data[0:2] != b'\x01\x01' or data[2:4] != b'\xf0\xff':
        return None
    return (data[11] << 8) | data[12]



def read_distance(ser):
    ser.reset_input_buffer()
    ser.write(POLL_CMD)
    response = ser.read(17)
    raw = parse_baumer_response(response)
    if raw is None:
        return None
    meas_mm = raw * RAW_TO_MM
    real_mm = calibrate(meas_mm)
    return raw, meas_mm, real_mm


# ============ Nucleo ============

def find_nucleo_port(exclude=None):
    for port in serial.tools.list_ports.comports():
        if exclude and port.device == exclude:
            continue
        if port.vid == 0x0483:
            return port.device
    for port in serial.tools.list_ports.comports():
        if exclude and port.device == exclude:
            continue
        desc = (port.description or '').lower()
        if any(k in desc for k in ('stm', 'nucleo', 'stlink')):
            return port.device
    return None


def connect_nucleo():
    while True:
        port = find_nucleo_port(exclude=BAUMER_PORT)
        if port is None:
            print('✗ Nucleo not found, retrying in 2 s...')
            time.sleep(2)
            continue
        try:
            s = serial.Serial(port, NUCLEO_BAUD, timeout=0.01)
            print(f'✓ Nucleo on {port}')
            return s
        except Exception as e:
            print(f'✗ Cannot open {port}: {e}')
            time.sleep(2)


def send_pwm(nucleo, pwm):
    """Send direct PWM command. Used by Mode 2."""
    if pwm < 0:
        pwm = 0
    elif pwm > PWM_MAX:
        pwm = PWM_MAX
    data = bytes([ord('P'), pwm & 0xFF, (pwm >> 8) & 0xFF])
    try:
        nucleo.write(data)
        return nucleo
    except (serial.SerialException, OSError):
        print('\nNucleo dropped, reconnecting...')
        try: nucleo.close()
        except Exception: pass
        nucleo = connect_nucleo()
        try: nucleo.write(data)
        except Exception as e: print(f'  write fail: {e}')
        return nucleo


def send_current(nucleo, amps):
    """Send current setpoint in amps (firmware expects mA). Modes 1, 3, 4."""
    ma = int(round(amps * 1000))
    if ma < 0:
        ma = 0
    # Clamp to MAX_COIL_CURRENT_A — the prior 4095 ceiling was a stale 12-bit
    # PWM limit. mA fits cleanly in the 16-bit protocol field (max 65535) and
    # the firmware enforces its own MAX_TARGET_MA = 5000 ceiling on top of this.
    max_ma = int(MAX_COIL_CURRENT_A * 1000)
    if ma > max_ma:
        ma = max_ma
    data = bytes([ord('I'), ma & 0xFF, (ma >> 8) & 0xFF])
    try:
        nucleo.write(data)
        return nucleo
    except (serial.SerialException, OSError):
        print('\nNucleo dropped, reconnecting...')
        try: nucleo.close()
        except Exception: pass
        nucleo = connect_nucleo()
        try: nucleo.write(data)
        except Exception as e: print(f'  write fail: {e}')
        return nucleo


















def mode_pid_control(nucleo, reader):
    print('\n--- Mode 1: Maglev position control (feedforward + PID) ---')
    print(f'  m={MASS_KG} kg | K(z) LUT spans {K_LUT_Z_MM[0]:.1f}–{K_LUT_Z_MM[-1]:.1f} mm '
          f'({len(K_LUT_Z_MM)} pts) | setpoint={SETPOINT_MM} mm')
    print(f'  PID: Kp={KP_POS}, Ki={KI_POS}, Kd={KD_POS} | loop={LOOP_HZ} Hz')
    print('  Ctrl+C → set new height (Ctrl+C again at prompt → exit)\n')

    setpoint_active = float(SETPOINT_MM)
    ramp_start_mm   = setpoint_active
    ramp_target_mm  = setpoint_active
    ramp_t0         = 0.0
    ramp_duration   = 0.0   # 0 = no ramp in progress

    try:
        baumer = serial.Serial(BAUMER_PORT, BAUMER_BAUD, timeout=0.05)
    except (serial.SerialException, OSError) as e:
        print(f'✗ Cannot open Baumer: {e}')
        input('  Press Enter to return to menu...')
        return nucleo

    integral = 0.0
    prev_error = 0.0
    filtered_derivative = 0.0
    last_time = time.time()
    loop_period = 1.0 / LOOP_HZ

    z_min_allowed = float(K_LUT_Z_MM[0])
    z_max_allowed = float(K_LUT_Z_MM[-1])

    # Display state — initialized so the status print is safe even if we
    # haven't successfully read distance yet on the first iteration.
    z, error, i_eq, i_pid, i_total = float('nan'), 0.0, 0.0, 0.0, 0.0

    # --- Non-blocking prompt state machine. ---
    # The control loop NEVER pauses. Keys are collected one at a time via
    # msvcrt; the ramp params only take effect when the user hits Enter on
    # the duration prompt. This is non-negotiable for stability: with
    # FF_TRIM > 1, sustaining the same current command for more than ~50 ms
    # without distance-corrected updates lets the magnet wind up and clamp.
    PROMPT_IDLE, PROMPT_SETPOINT, PROMPT_DURATION = 0, 1, 2
    prompt_state = PROMPT_IDLE
    input_buffer = ''
    pending_new_sp = 0.0
    exit_requested = False

    # Catch Ctrl+C as a flag rather than letting it raise KeyboardInterrupt
    # (which would abort the control loop, with disastrous results).
    ctrl_c_flag = [False]
    def _sigint(sig, frame):
        ctrl_c_flag[0] = True
    old_sigint = signal.signal(signal.SIGINT, _sigint)

    try:
        while True:
            cycle_start = time.time()

            # ============ Always run control ============
            result = read_distance(baumer)
            have_new_data = result is not None

            if have_new_data:
                raw, meas_mm, z = result

                now = time.time()
                dt = now - last_time
                last_time = now
                if dt <= 0 or dt > 0.5:
                    dt = loop_period

                # Advance ramp
                if ramp_duration > 0.0:
                    elapsed_ramp = now - ramp_t0
                    if elapsed_ramp >= ramp_duration:
                        setpoint_active = ramp_target_mm
                        ramp_duration = 0.0
                    else:
                        frac = elapsed_ramp / ramp_duration
                        setpoint_active = ramp_start_mm + frac * (ramp_target_mm - ramp_start_mm)
                else:
                    setpoint_active = ramp_target_mm

                k_local = k_at(z)
                i_eq = z * np.sqrt(MASS_KG * G / k_local) * FF_TRIM / 1000.0
                error = z - setpoint_active

                if z <= MAX_USEFUL_GAP:
                    raw_derivative = (error - prev_error) / dt
                    alpha = np.exp(-dt / DERIV_FILTER_TAU)
                    filtered_derivative = (alpha * filtered_derivative
                                           + (1.0 - alpha) * raw_derivative)

                    i_pid_provisional = (KP_POS * error
                                         + KI_POS * integral
                                         + KD_POS * filtered_derivative)
                    i_total_provisional = i_eq + i_pid_provisional

                    saturating_high = (i_total_provisional >= MAX_COIL_CURRENT_A
                                       and error > 0)
                    saturating_low  = (i_total_provisional <= 0.0
                                       and error < 0)
                    if not saturating_high and not saturating_low:
                        integral += error * dt
                        if integral >  INTEGRAL_LIMIT: integral =  INTEGRAL_LIMIT
                        if integral < -INTEGRAL_LIMIT: integral = -INTEGRAL_LIMIT
                else:
                    filtered_derivative = 0.0
                prev_error = error

                i_pid = KP_POS * error + KI_POS * integral + KD_POS * filtered_derivative

                i_total = i_eq + i_pid
                if i_total < 0.0: i_total = 0.0
                if i_total > MAX_COIL_CURRENT_A: i_total = MAX_COIL_CURRENT_A
                nucleo = send_current(nucleo, i_total)

            # ============ Handle Ctrl+C flag ============
            if ctrl_c_flag[0]:
                ctrl_c_flag[0] = False
                if prompt_state == PROMPT_IDLE:
                    sys.stdout.write('\n')
                    sys.stdout.write(
                        f'  New setpoint mm [Enter=resume, Ctrl+C=exit, '
                        f'current={setpoint_active:.2f}]: ')
                    sys.stdout.flush()
                    input_buffer = ''
                    prompt_state = PROMPT_SETPOINT
                else:
                    exit_requested = True

            # ============ Handle keystrokes (non-blocking) ============
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                # Discard 2-byte special keys (function keys, arrows)
                if ch in (b'\x00', b'\xe0'):
                    if msvcrt.kbhit():
                        msvcrt.getch()
                    continue
                # Some terminals deliver Ctrl+C as 0x03 here instead of SIGINT
                if ch == b'\x03':
                    ctrl_c_flag[0] = True
                    continue
                if ch in (b'\r', b'\n'):
                    if prompt_state == PROMPT_IDLE:
                        continue  # ignore stray Enter outside the prompt
                    sys.stdout.write('\n')
                    sys.stdout.flush()
                    if prompt_state == PROMPT_SETPOINT:
                        if input_buffer == '':
                            print('  (no change)')
                            prompt_state = PROMPT_IDLE
                        else:
                            try:
                                new_sp = float(input_buffer)
                                if new_sp < z_min_allowed or new_sp > z_max_allowed:
                                    print(f'  ✗ Setpoint must be in '
                                          f'[{z_min_allowed:.2f}, {z_max_allowed:.2f}] mm '
                                          f'(K-LUT range).')
                                    prompt_state = PROMPT_IDLE
                                else:
                                    pending_new_sp = new_sp
                                    sys.stdout.write(
                                        f'  Duration to ramp from {setpoint_active:.2f} → '
                                        f'{new_sp:.2f} mm (seconds, 0 = instant): ')
                                    sys.stdout.flush()
                                    prompt_state = PROMPT_DURATION
                            except ValueError:
                                print('  ✗ Invalid number.')
                                prompt_state = PROMPT_IDLE
                    elif prompt_state == PROMPT_DURATION:
                        try:
                            duration = float(input_buffer) if input_buffer else 0.0
                            if duration < 0:
                                duration = 0.0
                            ramp_start_mm  = setpoint_active
                            ramp_target_mm = pending_new_sp
                            ramp_t0        = time.time()
                            ramp_duration  = duration
                            if duration == 0.0:
                                setpoint_active = pending_new_sp
                                print(f'  → Setpoint snapped to {pending_new_sp:.2f} mm.')
                            else:
                                print(f'  → Ramping {ramp_start_mm:.2f} → '
                                      f'{pending_new_sp:.2f} mm over {duration:.1f} s.')
                        except ValueError:
                            print('  ✗ Invalid number — no ramp applied.')
                        prompt_state = PROMPT_IDLE
                    input_buffer = ''
                elif ch == b'\x08':  # Backspace
                    if prompt_state != PROMPT_IDLE and input_buffer:
                        input_buffer = input_buffer[:-1]
                        sys.stdout.write('\b \b')
                        sys.stdout.flush()
                elif prompt_state != PROMPT_IDLE:
                    try:
                        c = ch.decode('ascii')
                        if 32 <= ord(c) < 127:
                            input_buffer += c
                            sys.stdout.write(c)
                            sys.stdout.flush()
                    except (UnicodeDecodeError, ValueError):
                        pass
                # Outside a prompt, non-Enter non-Ctrl-C keys are simply ignored.

            # ============ Status display (only while idle) ============
            if prompt_state == PROMPT_IDLE and have_new_data:
                meas = reader.latest_a
                meas_part = f'  I_meas={meas:.2f}A' if meas is not None else ''
                ramp_part = ''
                if ramp_duration > 0.0:
                    remaining = ramp_duration - (time.time() - ramp_t0)
                    ramp_part = f'  →{ramp_target_mm:.1f}mm in {max(0.0, remaining):4.1f}s'
                sys.stdout.write(
                    f'\rz={z:6.2f}mm  sp={setpoint_active:5.2f}mm  '
                    f'err={error:+6.2f}mm  '
                    f'I_eq={i_eq:5.2f}  I_pid={i_pid:+5.2f}  '
                    f'I*={i_total:5.2f}A{meas_part}{ramp_part}  ')
                sys.stdout.flush()

            # ============ Exit if requested ============
            if exit_requested:
                print('\n  Exiting Mode 1. Sending zero current.')
                nucleo = send_current(nucleo, 0)
                break

            # ============ Pace the loop ============
            wall_elapsed = time.time() - cycle_start
            sleep_for = loop_period - wall_elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    finally:
        signal.signal(signal.SIGINT, old_sigint)
        try:
            baumer.close()
        except Exception:
            pass

    return nucleo





















# ============ Mode 2: Manual PWM ============

def mode_manual_pwm(nucleo, reader):
    print('\n--- Mode 2: Manual PWM injection ---')
    print('Enter PWM value (0-4095). Commands: "z" = zero now, "q" = back to menu.')
    print('Magnet holds whatever value you last entered until you change it.\n')

    current_pwm = 0
    nucleo = send_pwm(nucleo, current_pwm)
    print('  PWM zeroed.')

    while True:
        try:
            user_input = input(f'PWM [current={current_pwm}]: ').strip().lower()
        except (KeyboardInterrupt, EOFError):
            print()
            break

        if user_input == 'q':
            break
        if user_input == '':
            continue
        if user_input == 'z':
            current_pwm = 0
            nucleo = send_pwm(nucleo, current_pwm)
            time.sleep(0.05)
            meas = reader.latest_a
            meas_str = f'  meas={meas:.3f}A' if meas is not None else '  meas=?'
            print(f'  → 0{meas_str}')
            continue

        try:
            pwm = int(user_input)
        except ValueError:
            print('  Invalid. Enter an integer 0-4095, "z", or "q".')
            continue
        if pwm < 0 or pwm > PWM_MAX:
            print(f'  Out of range. Use 0-{PWM_MAX}.')
            continue

        current_pwm = pwm
        nucleo = send_pwm(nucleo, current_pwm)
        time.sleep(0.25)                       # let ~12 fresh samples arrive
        meas, std = reader.stats(n=10)
        if meas is None:
            meas_str = '  meas=?'
        else:
            meas_str = f'  meas={meas:.3f}A  ±{std:.3f}'
        print(f'  → {pwm}{meas_str}')

    # Safety: zero on exit
    nucleo = send_pwm(nucleo, 0)
    print('  Returning to menu (PWM=0).')
    return nucleo








# ============ Mode 3: Coil-current-controlled PWM ============

def mode_coil_current_input(nucleo, reader):
    print('\n--- Mode 3: Coil-current controlled PWM (with step-response diagnostics) ---')
    print(f'Enter desired coil current (0.000 to {MAX_COIL_CURRENT_A:.3f} A).')
    print('Each entry captures a step response (latency + 10-90% rise time)')
    print('followed by a live readout.')
    print('Commands: "z" = zero now, "q" = back to menu.\n')

    # Optional Baumer for live distance readout — non-fatal if unavailable
    baumer = None
    try:
        baumer = serial.Serial(BAUMER_PORT, BAUMER_BAUD, timeout=0.05)
        print(f'  Baumer opened on {BAUMER_PORT}; distance will display live.')
    except (serial.SerialException, OSError) as e:
        print(f'  ⚠ Baumer unavailable ({e}); proceeding without distance.')

    set_amps = 0.0
    set_pwm = 0
    nucleo = send_current(nucleo, set_amps)
    print('  Output zeroed.')

    while True:
        try:
            user_input = input(
                f'\nI_coil [setpoint={set_amps:.3f} A → PWM={set_pwm}]: '
            ).strip().lower()
        except (KeyboardInterrupt, EOFError):
            print()
            break

        if user_input == 'q':
            break
        if user_input == '':
            continue

        if user_input == 'z':
            new_pwm = 0
            new_amps = 0.0
        else:
            try:
                amps = float(user_input)
            except ValueError:
                print('  Invalid. Enter a number, "z", or "q".')
                continue
            if amps < 0:
                print('  Negative current not allowed.')
                continue
            if amps > MAX_COIL_CURRENT_A:
                print(f'  Clipping to {MAX_COIL_CURRENT_A:.3f} A')
                amps = MAX_COIL_CURRENT_A
            new_pwm = coil_current_to_pwm(amps)
            new_amps = amps

        # === Step response capture ===
        nucleo, r = capture_step_response(nucleo, reader, new_amps)
        set_amps = new_amps
        set_pwm  = new_pwm

        baseline = r['baseline']
        final    = r['final']
        latency  = r['latency']
        rise     = r['rise_time']

        if final is None:
            print(f'  → PWM={set_pwm}: no samples received during capture')
        else:
            err = final - set_amps
            print(f'\n  Step: {baseline:.3f} A → {final:.3f} A   '
                  f'(target {set_amps:.3f} A, err {err:+.3f} A)')
            print(f'  Samples captured: {len(r["samples"])}')

            if latency is not None:
                print(f'  Latency (5% threshold):  {latency*1000:6.1f} ms')
            else:
                print(f'  Latency:    (step too small or no crossing seen)')

            if rise is not None:
                print(f'  Rise time (10-90%):      {rise*1000:6.1f} ms')
            else:
                print(f'  Rise time:  (could not resolve)')

        # === Live readout — runs until Ctrl+C, shows z if Baumer is connected ===
        show_live_readout(reader, duration=None, baumer=baumer)

    nucleo = send_current(nucleo, 0)
    if baumer is not None:
        try:
            baumer.close()
        except Exception:
            pass
    print('  Returning to menu (zero current).')
    return nucleo









def mode_k_calibration(nucleo, reader):
    print('\n--- Mode 4: K(z) calibration ---')
    print('Static measurement with live Baumer z. Magnet on scale below plate.')
    print(f'Static magnet mass: {MASS_KG:.3f} kg → no-current scale = {MASS_KG:.3f} kg\n')
    print('Per measurement:')
    print('  1) Position phase — live z displayed, no current. Ctrl+C when set.')
    print('  2) Confirm (Enter=apply, s=skip, q=quit).')
    print('  3) Capture phase — current on, live z keeps streaming and averaging.')
    print('     Read your scale. Ctrl+C when reading is stable.')
    print('  4) Type kg reading → K computed → current drops.\n')

    try:
        baumer = serial.Serial(BAUMER_PORT, BAUMER_BAUD, timeout=0.05)
    except (serial.SerialException, OSError) as e:
        print(f'✗ Cannot open Baumer: {e}')
        return nucleo

    nucleo = send_current(nucleo, 0)  # ensure idle on entry

    try:
        amps_in = input('Calibration current [A, default 2.0]: ').strip().lower()
        if amps_in == 'q':
            baumer.close()
            return nucleo
        amps = float(amps_in) if amps_in else 2.0
        if amps <= 0 or amps > MAX_COIL_CURRENT_A:
            amps = 2.0
            print(f'  Out of range; using {amps:.3f} A')
    except (ValueError, KeyboardInterrupt, EOFError):
        amps = 2.0

    measurements = []  # list of (z_mm, K, scale_kg, F_N)

    try:
        while True:
            # === Phase 1: position the magnet (no current) ===
            print('\n  === New measurement ===')
            print('  Position phase: adjust spacers. Ctrl+C when set.')
            try:
                while True:
                    r = read_distance(baumer)
                    z_str = f'{r[2]:6.2f} mm' if r is not None else '   ?    '
                    print(f'\r    z = {z_str}        ', end='', flush=True)
                    time.sleep(0.05)
            except KeyboardInterrupt:
                print()

            # === Phase 2: confirm before applying current ===
            try:
                go = input(f'  Apply I={amps:.3f} A? [Enter=yes, s=skip, q=quit]: ').strip().lower()
            except (KeyboardInterrupt, EOFError):
                go = 'q'
            if go == 'q':
                break
            if go == 's':
                continue

            # === Phase 3: capture — current on, live z + averaging ===
            nucleo = send_current(nucleo, amps)
            time.sleep(0.3)  # let current settle before we start averaging

            print('  Capture phase: current ON. Read scale; Ctrl+C when stable.')
            z_samples = []
            try:
                while True:
                    r = read_distance(baumer)
                    if r is not None:
                        z_samples.append(r[2])
                        live = f'{r[2]:6.2f}'
                    else:
                        live = '  ?  '
                    n = len(z_samples)
                    print(f'\r    z = {live} mm   ({n} samples)        ',
                          end='', flush=True)
                    time.sleep(0.05)
            except KeyboardInterrupt:
                print()

            # === Phase 4: prompt for scale reading, drop current ===
            try:
                scale_in = input('  Scale reading (kg) [Enter/s=skip, q=quit]: ').strip().lower()
            except (KeyboardInterrupt, EOFError):
                scale_in = 'q'
            nucleo = send_current(nucleo, 0)  # always drop current on exit from capture

            if scale_in == 'q':
                break
            if scale_in in ('s', ''):
                print('  Skipped (no scale value entered).')
                continue
            if not z_samples:
                print('  ✗ No z samples captured during phase 3 — skipping.')
                continue

            try:
                scale_kg = float(scale_in)
            except ValueError:
                print('  Invalid reading; skipping.')
                continue

            z_mean = sum(z_samples) / len(z_samples)
            z_std  = (sum((zz - z_mean) ** 2 for zz in z_samples) / len(z_samples)) ** 0.5
            f_lift = (MASS_KG - scale_kg) * G
            if f_lift <= 0:
                print(f'  ⚠ Scale ≥ static mass ({scale_kg:.3f} ≥ {MASS_KG:.3f}). '
                      f'No upward force. Skipping.')
                continue

            z_m = z_mean / 1000.0
            k = f_lift * z_m * z_m / (amps * amps)
            measurements.append((z_mean, k, scale_kg, f_lift))
            print(f'  → z={z_mean:.2f} ±{z_std:.2f} mm  '
                  f'F={f_lift:.2f} N  K={k:.6f} N·m²/A²')

    except KeyboardInterrupt:
        print()
    finally:
        nucleo = send_current(nucleo, 0)
        try:
            baumer.close()
        except Exception:
            pass

    if not measurements:
        print('  No measurements taken.')
        return nucleo

    measurements.sort(key=lambda m: m[0])
    print('\n  === Summary (sorted by z) ===')
    print('     z (mm)        K        scale (kg)   F_lift (N)')
    for z, k, s, f in measurements:
        print(f'    {z:7.2f}   {k:.6f}    {s:7.3f}     {f:7.2f}')

    print('\n  === Paste into Python config ===')
    z_str = ',\n    '.join(f'{m[0]:6.2f}' for m in measurements)
    k_str = ',\n    '.join(f'{m[1]:.6f}' for m in measurements)
    print(f'  K_LUT_Z_MM = np.array([\n    {z_str},\n  ], dtype=float)')
    print(f'  K_LUT_VALUES = np.array([\n    {k_str},\n  ], dtype=float)')

    return nucleo






def mode_k_auto_calibration(nucleo, reader):
    print('\n--- Mode 5: K(z) auto-calibration (closed-loop hover) ---')
    print(f'  m={MASS_KG} kg | K(z) LUT spans {K_LUT_Z_MM[0]:.1f}-{K_LUT_Z_MM[-1]:.1f} mm '
          f'({len(K_LUT_Z_MM)} pts)')
    print('  Hovers at requested heights, measures K = mg·z²/I² at steady state.')
    print('  Ctrl+C → set new measurement height (Ctrl+C again at prompt → exit)\n')

    setpoint_active = float(SETPOINT_MM)
    ramp_start_mm   = setpoint_active
    ramp_target_mm  = setpoint_active
    ramp_t0         = 0.0
    ramp_duration   = 0.0

    try:
        baumer = serial.Serial(BAUMER_PORT, BAUMER_BAUD, timeout=0.05)
    except (serial.SerialException, OSError) as e:
        print(f'✗ Cannot open Baumer: {e}')
        input('  Press Enter to return to menu...')
        return nucleo

    integral = 0.0
    prev_error = 0.0
    filtered_derivative = 0.0
    last_time = time.time()
    loop_period = 1.0 / LOOP_HZ

    z_min_allowed = float(K_LUT_Z_MM[0])
    z_max_allowed = float(K_LUT_Z_MM[-1])

    z, error, i_eq, i_pid, i_total = float('nan'), 0.0, 0.0, 0.0, 0.0

    # --- Measurement state machine ---
    MEAS_IDLE, MEAS_WAITING, MEAS_ACTIVE = 0, 1, 2
    meas_state = MEAS_IDLE
    meas_t0 = 0.0
    meas_z_samples = []
    meas_i_samples = []
    measurements = []   # list of (z_mean, k, i_mean, z_std, i_std)

    win_len = max(50, int(1.5 * LOOP_HZ))
    z_window = deque(maxlen=win_len)
    i_window = deque(maxlen=win_len)

    STABILITY_Z_STD  = 0.15      # mm
    STABILITY_I_STD  = 0.15      # A
    STABILITY_SP_TOL = 0.8       # mm — looser since LUT may be wrong
    MEAS_DURATION_S  = 3.0
    MEAS_TIMEOUT_S   = 30.0

    PROMPT_IDLE, PROMPT_SETPOINT, PROMPT_DURATION = 0, 1, 2
    prompt_state = PROMPT_IDLE
    input_buffer = ''
    pending_new_sp = 0.0
    exit_requested = False

    ctrl_c_flag = [False]
    def _sigint(sig, frame):
        ctrl_c_flag[0] = True
    old_sigint = signal.signal(signal.SIGINT, _sigint)

    try:
        while True:
            cycle_start = time.time()

            # ============ Control ============
            result = read_distance(baumer)
            have_new_data = result is not None

            if have_new_data:
                raw, meas_mm, z = result
                now = time.time()
                dt = now - last_time
                last_time = now
                if dt <= 0 or dt > 0.5:
                    dt = loop_period

                if ramp_duration > 0.0:
                    elapsed_ramp = now - ramp_t0
                    if elapsed_ramp >= ramp_duration:
                        setpoint_active = ramp_target_mm
                        ramp_duration = 0.0
                    else:
                        frac = elapsed_ramp / ramp_duration
                        setpoint_active = ramp_start_mm + frac * (ramp_target_mm - ramp_start_mm)
                else:
                    setpoint_active = ramp_target_mm

                k_local = k_at(z)
                i_eq = z * np.sqrt(MASS_KG * G / k_local) * FF_TRIM / 1000.0
                error = z - setpoint_active

                if z <= MAX_USEFUL_GAP:
                    raw_derivative = (error - prev_error) / dt
                    alpha = np.exp(-dt / DERIV_FILTER_TAU)
                    filtered_derivative = (alpha * filtered_derivative
                                           + (1.0 - alpha) * raw_derivative)
                    i_pid_provisional = (KP_POS * error + KI_POS * integral
                                         + KD_POS * filtered_derivative)
                    i_total_provisional = i_eq + i_pid_provisional
                    saturating_high = (i_total_provisional >= MAX_COIL_CURRENT_A and error > 0)
                    saturating_low  = (i_total_provisional <= 0.0 and error < 0)
                    if not saturating_high and not saturating_low:
                        integral += error * dt
                        if integral >  INTEGRAL_LIMIT: integral =  INTEGRAL_LIMIT
                        if integral < -INTEGRAL_LIMIT: integral = -INTEGRAL_LIMIT
                else:
                    filtered_derivative = 0.0
                prev_error = error

                i_pid = KP_POS * error + KI_POS * integral + KD_POS * filtered_derivative
                i_total = i_eq + i_pid
                if i_total < 0.0: i_total = 0.0
                if i_total > MAX_COIL_CURRENT_A: i_total = MAX_COIL_CURRENT_A
                nucleo = send_current(nucleo, i_total)

                z_window.append(z)
                i_now = reader.latest_a
                if i_now is not None and i_now > 0:
                    i_window.append(i_now)

            # ============ Measurement state machine ============
            if meas_state == MEAS_WAITING and have_new_data and ramp_duration == 0.0:
                if len(z_window) >= win_len and len(i_window) >= win_len // 2:
                    z_arr = list(z_window)
                    z_mu = sum(z_arr) / len(z_arr)
                    z_sd = (sum((zz - z_mu) ** 2 for zz in z_arr) / len(z_arr)) ** 0.5
                    i_arr = list(i_window)
                    i_mu = sum(i_arr) / len(i_arr)
                    i_sd = (sum((ii - i_mu) ** 2 for ii in i_arr) / len(i_arr)) ** 0.5

                    if (z_sd < STABILITY_Z_STD
                        and i_sd < STABILITY_I_STD
                        and abs(z_mu - setpoint_active) < STABILITY_SP_TOL):
                        sys.stdout.write(
                            f'\n  ✓ Stable at z={z_mu:.2f}mm (z_sd={z_sd:.3f}, i_sd={i_sd:.3f}). '
                            f'Measuring {MEAS_DURATION_S:.1f}s...\n')
                        sys.stdout.flush()
                        meas_state = MEAS_ACTIVE
                        meas_t0 = time.time()
                        meas_z_samples = []
                        meas_i_samples = []

                if time.time() - meas_t0 > MEAS_TIMEOUT_S:
                    sys.stdout.write(f'\n  ✗ Did not stabilize within {MEAS_TIMEOUT_S:.0f}s. Aborting.\n')
                    sys.stdout.flush()
                    meas_state = MEAS_IDLE

            if meas_state == MEAS_ACTIVE and have_new_data:
                meas_z_samples.append(z)
                i_now = reader.latest_a
                if i_now is not None and i_now > 0:
                    meas_i_samples.append(i_now)

                if time.time() - meas_t0 > MEAS_DURATION_S:
                    if len(meas_z_samples) >= 10 and len(meas_i_samples) >= 10:
                        z_m  = sum(meas_z_samples) / len(meas_z_samples)
                        z_sd = (sum((zz - z_m) ** 2 for zz in meas_z_samples) / len(meas_z_samples)) ** 0.5
                        i_m  = sum(meas_i_samples) / len(meas_i_samples)
                        i_sd = (sum((ii - i_m) ** 2 for ii in meas_i_samples) / len(meas_i_samples)) ** 0.5
                        z_meters = z_m / 1000.0
                        k_new = MASS_KG * G * z_meters * z_meters / (i_m * i_m)
                        k_old = k_at(z_m)
                        delta_pct = 100.0 * (k_new - k_old) / k_old
                        measurements.append((z_m, k_new, i_m, z_sd, i_sd))
                        sys.stdout.write(
                            f'\n  → z={z_m:6.2f} ±{z_sd:.3f}mm  '
                            f'I={i_m:.3f} ±{i_sd:.3f}A  '
                            f'K={k_new:.6f}  '
                            f'(LUT={k_old:.6f}, Δ={delta_pct:+.1f}%)\n')
                        sys.stdout.flush()
                    else:
                        sys.stdout.write('\n  ✗ Insufficient samples collected.\n')
                        sys.stdout.flush()
                    meas_state = MEAS_IDLE

            # ============ Ctrl+C ============
            if ctrl_c_flag[0]:
                ctrl_c_flag[0] = False
                if prompt_state == PROMPT_IDLE:
                    sys.stdout.write('\n')
                    sys.stdout.write(
                        f'  Target z to measure (mm, {z_min_allowed:.1f}–{z_max_allowed:.1f}) '
                        f'[Enter=cancel, Ctrl+C=exit]: ')
                    sys.stdout.flush()
                    input_buffer = ''
                    prompt_state = PROMPT_SETPOINT
                else:
                    exit_requested = True

            # ============ Keyboard ============
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x00', b'\xe0'):
                    if msvcrt.kbhit():
                        msvcrt.getch()
                    continue
                if ch == b'\x03':
                    ctrl_c_flag[0] = True
                    continue
                if ch in (b'\r', b'\n'):
                    if prompt_state == PROMPT_IDLE:
                        continue
                    sys.stdout.write('\n')
                    sys.stdout.flush()
                    if prompt_state == PROMPT_SETPOINT:
                        if input_buffer == '':
                            sys.stdout.write('  (no change)\n')
                            prompt_state = PROMPT_IDLE
                        else:
                            try:
                                new_sp = float(input_buffer)
                                if new_sp < z_min_allowed or new_sp > z_max_allowed:
                                    sys.stdout.write(
                                        f'  ✗ Must be in [{z_min_allowed:.2f}, '
                                        f'{z_max_allowed:.2f}] mm.\n')
                                    prompt_state = PROMPT_IDLE
                                else:
                                    pending_new_sp = new_sp
                                    sys.stdout.write(
                                        f'  Ramp duration {setpoint_active:.2f} → {new_sp:.2f} mm '
                                        f'[seconds, default=2]: ')
                                    sys.stdout.flush()
                                    prompt_state = PROMPT_DURATION
                            except ValueError:
                                sys.stdout.write('  ✗ Invalid number.\n')
                                prompt_state = PROMPT_IDLE
                    elif prompt_state == PROMPT_DURATION:
                        try:
                            duration = float(input_buffer) if input_buffer else 2.0
                            if duration < 0:
                                duration = 0.0
                            ramp_start_mm  = setpoint_active
                            ramp_target_mm = pending_new_sp
                            ramp_t0        = time.time()
                            ramp_duration  = duration
                            meas_state = MEAS_WAITING
                            meas_t0    = time.time() + duration
                            z_window.clear()
                            i_window.clear()
                            if duration == 0.0:
                                setpoint_active = pending_new_sp
                                sys.stdout.write(
                                    f'  → Snap to {pending_new_sp:.2f} mm; measuring once stable.\n')
                            else:
                                sys.stdout.write(
                                    f'  → Ramp over {duration:.1f}s, then measure.\n')
                            sys.stdout.flush()
                        except ValueError:
                            sys.stdout.write('  ✗ Invalid number.\n')
                        prompt_state = PROMPT_IDLE
                    input_buffer = ''
                elif ch == b'\x08':
                    if prompt_state != PROMPT_IDLE and input_buffer:
                        input_buffer = input_buffer[:-1]
                        sys.stdout.write('\b \b')
                        sys.stdout.flush()
                elif prompt_state != PROMPT_IDLE:
                    try:
                        c = ch.decode('ascii')
                        if 32 <= ord(c) < 127:
                            input_buffer += c
                            sys.stdout.write(c)
                            sys.stdout.flush()
                    except (UnicodeDecodeError, ValueError):
                        pass

            # ============ Status line ============
            if prompt_state == PROMPT_IDLE and have_new_data:
                meas_str = ''
                if meas_state == MEAS_WAITING:
                    if len(z_window) >= win_len and len(i_window) >= win_len // 2:
                        z_arr = list(z_window); i_arr = list(i_window)
                        z_mu = sum(z_arr)/len(z_arr)
                        z_sd = (sum((zz - z_mu)**2 for zz in z_arr) / len(z_arr))**0.5
                        i_mu = sum(i_arr)/len(i_arr)
                        i_sd = (sum((ii - i_mu)**2 for ii in i_arr) / len(i_arr))**0.5
                        meas_str = f'  WAIT z_sd={z_sd:.3f} i_sd={i_sd:.3f}'
                    else:
                        meas_str = '  WAIT (filling window)'
                elif meas_state == MEAS_ACTIVE:
                    progress = (time.time() - meas_t0) / MEAS_DURATION_S
                    meas_str = f'  MEAS [{int(progress*100):3d}%]'

                ramp_part = ''
                if ramp_duration > 0.0:
                    remaining = ramp_duration - (time.time() - ramp_t0)
                    ramp_part = f'  →{ramp_target_mm:.1f}mm in {max(0.0, remaining):4.1f}s'

                sys.stdout.write(
                    f'\rz={z:6.2f}mm  sp={setpoint_active:5.2f}  '
                    f'err={error:+6.2f}  I*={i_total:5.2f}A'
                    f'{ramp_part}{meas_str}  ')
                sys.stdout.flush()

            if exit_requested:
                sys.stdout.write('\n  Exiting Mode 5. Sending zero current.\n')
                sys.stdout.flush()
                nucleo = send_current(nucleo, 0)
                break

            wall_elapsed = time.time() - cycle_start
            sleep_for = loop_period - wall_elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    finally:
        signal.signal(signal.SIGINT, old_sigint)
        try:
            baumer.close()
        except Exception:
            pass

    if not measurements:
        print('  No measurements taken.')
        return nucleo

    measurements.sort(key=lambda m: m[0])
    print('\n  === Summary (sorted by z) ===')
    print('     z (mm)     I (A)       K           z_std    i_std    vs LUT')
    for z_m, k, i_m, z_s, i_s in measurements:
        k_old = k_at(z_m)
        delta_pct = 100.0 * (k - k_old) / k_old
        print(f'    {z_m:7.2f}   {i_m:7.3f}   {k:.6f}    ±{z_s:.3f}  ±{i_s:.3f}A  ({delta_pct:+5.1f}%)')

    print('\n  === Paste into Python config ===')
    z_str = ',\n    '.join(f'{m[0]:6.2f}' for m in measurements)
    k_str = ',\n    '.join(f'{m[1]:.6f}' for m in measurements)
    print(f'  K_LUT_Z_MM = np.array([\n    {z_str},\n  ], dtype=float)')
    print(f'  K_LUT_VALUES = np.array([\n    {k_str},\n  ], dtype=float)')

    return nucleo









# ============ Menu ============

def menu():
    print('\n' + '=' * 50)
    print('  Maglev Controller')
    print('=' * 50)
    print('  1. PID closed-loop control (sensor → PWM)')
    print('  2. Manual PWM injection (raw)')
    print('  3. Coil-current PWM (target coil amps)')
    print('  4. K(z) calibration (static, with scale)')
    print('  5. K(z) auto-calibration (closed-loop hover, no scale)')
    print('  q. Quit')
    print('=' * 50)
    try:
        return input('Choice: ').strip().lower()
    except (KeyboardInterrupt, EOFError):
        return 'q'


def main():
    print('Connecting to Nucleo...')
    nucleo = connect_nucleo()
    reader = CurrentReader(nucleo)

    while True:
        choice = menu()
        if choice == '1':
            nucleo = mode_pid_control(nucleo, reader)
        elif choice == '2':
            nucleo = mode_manual_pwm(nucleo, reader)
        elif choice == '3':
            nucleo = mode_coil_current_input(nucleo, reader)
        elif choice == '4':
            nucleo = mode_k_calibration(nucleo, reader)
        elif choice == '5':
            nucleo = mode_k_auto_calibration(nucleo, reader)
        elif choice in ('q', 'quit', 'exit'):
            break
        else:
            print(f'  Unknown choice: {choice!r}')

    reader.stop()
    print('\nShutting down. Sending zero current.')
    try:
        send_current(nucleo, 0)
        nucleo.close()
    except Exception:
        pass
    ctypes.windll.winmm.timeEndPeriod(1)
    print('Done.')


if __name__ == '__main__':
    main()


