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
ctypes.windll.winmm.timeBeginPeriod(1)   # Windows: set system timer to 1 ms resolution


# ============ Configuration ============
BAUMER_PORT = 'COM10'
BAUMER_BAUD = 115200
NUCLEO_BAUD = 115200
PWM_MAX = 4095   
# === Maglev physics — z is in METERS in F = K·(I/z)² ===
MASS_KG    = 8.5          # measured with sensor and mounts attached
G          = 9.81

# === K(z) lookup table — N·m²/A² (z in mm) ===
# Measured statically with the scale:
#   F_lift = (MASS_KG - scale_kg) · g
#   K      = F_lift · (z/1000)² / I²
# Add rows as more data points become available. k_at() clamps to LUT
# range — don't extrapolate the magnetic model past where we've measured.
K_LUT_Z_MM = np.array([
      3.94,
      4.92,
      5.93,
      6.95,
      7.99,
      8.98,
     9.97,
     10.96,
  ], dtype=float)
K_LUT_VALUES = np.array([
    0.000378,
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

# === Position loop ===
SETPOINT_MM      = 7.0        # 5A limit forces us below ~9.3 mm for 8.5 kg
MAX_USEFUL_GAP   = 15        # above this, 5A can't hold the magnet — turn off


FF_TRIM          = 0.82      # keep
KP_POS           = 0.03       # A/mm — slightly less aggressive
KI_POS           = 0.00       # A/(mm·s) — less integrator authority needed with anti-windup
KD_POS           = 0.03      # A·s/mm — more damping to kill velocity at plate approaches
DERIV_FILTER_TAU = 0.020      # keep
INTEGRAL_LIMIT   = 10.0       # mm·s — Ki×LIMIT = 0.5 A integrator authority
LOOP_HZ          = 100










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
MAX_CURRENT_A = 5.0  

# Coil current (through magnet, measured by INA260) — used by Mode 4 and Mode 1 display
CAL_COIL_CURRENT = np.array([
    0.069, 0.162, 0.277, 0.398, 0.520, 0.630, 0.749, 0.876, 0.991, 1.096,
    1.211, 1.326, 1.442, 1.556, 1.668, 1.783, 1.896, 2.010, 2.116, 2.229,
    2.355, 2.476, 2.607, 2.746, 2.910, 3.101, 3.305, 3.778, 3.851,
], dtype=float)
MAX_COIL_CURRENT_A = 5.0


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
    elif ma > 4095:
        ma = 4095
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
    print('  Ctrl+C to return to menu\n')

    try:
        baumer = serial.Serial(BAUMER_PORT, BAUMER_BAUD, timeout=0.05)
    except (serial.SerialException, OSError) as e:
        print(f'✗ Cannot open Baumer: {e}')
        input('  Press Enter to return to menu...')
        return nucleo

    # FF is now z-dependent — K varies with gap, so we recompute each iteration.
    # Underlying physics: I_eq = z[m] · √(mg/K(z)). FF_TRIM<1 biases the FF to
    # undershoot, which keeps the magnet falling rather than clamping into the
    # plate during transients.

    integral = 0.0
    prev_error = 0.0
    filtered_derivative = 0.0     # IIR filter state for the D term
    last_time = time.time()
    loop_period = 1.0 / LOOP_HZ

    try:
        while True:
            cycle_start = time.time()

            result = read_distance(baumer)
            if result is None:
                continue
            raw, meas_mm, z = result   # z in mm

            now = time.time()
            dt = now - last_time
            last_time = now
            if dt <= 0 or dt > 0.5:
                dt = loop_period

            # Feedforward: current to balance gravity at CURRENT z.
            # When z is too far for the FF to fully cancel gravity, i_eq
            # exceeds MAX_COIL_CURRENT_A and the clamp at the bottom of
            # this block hands us "max current, try to grab it" behavior.
            k_local = k_at(z)
            i_eq = z * np.sqrt(MASS_KG * G / k_local) * FF_TRIM / 1000.0

            # Error convention: positive = too far (need more current)
            error = z - SETPOINT_MM

            # Only integrate when we're actually within control range —
            # otherwise the integrator winds up uselessly during the lift phase.
            if z <= MAX_USEFUL_GAP:
                raw_derivative = (error - prev_error) / dt
                alpha = np.exp(-dt / DERIV_FILTER_TAU)
                filtered_derivative = (alpha * filtered_derivative
                                       + (1.0 - alpha) * raw_derivative)

                # Compute provisional output with the CURRENT integrator state
                i_pid_provisional = (KP_POS * error
                                     + KI_POS * integral
                                     + KD_POS * filtered_derivative)
                i_total_provisional = i_eq + i_pid_provisional

                # Anti-windup: only integrate if doing so wouldn't worsen saturation
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

            meas = reader.latest_a
            meas_part = f'  I_meas={meas:.2f}A' if meas is not None else ''
            print(f'\rz={z:6.2f}mm  err={(z-SETPOINT_MM):+6.2f}mm  '
                  f'I_eq={i_eq:5.2f}  I_pid={i_pid:+5.2f}  '
                  f'I*={i_total:5.2f}A{meas_part}  ',
                  end='')

            elapsed = time.time() - cycle_start
            sleep_for = loop_period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print('\n  Stopping. Sending zero current.')
        nucleo = send_current(nucleo, 0)
    finally:
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
















# ============ Menu ============

def menu():
    print('\n' + '=' * 50)
    print('  Maglev Controller')
    print('=' * 50)
    print('  1. PID closed-loop control (sensor → PWM)')
    print('  2. Manual PWM injection (raw)')
    print('  3. Coil-current PWM (target coil amps)')
    print('  4. K(z) calibration (static, with scale)')
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