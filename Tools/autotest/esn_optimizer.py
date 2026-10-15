#!/usr/bin/env python3
"""
Grid search гиперпараметров ESN для ArduPlane (SITL) в режиме ACRO.

Запуск:
    1. В первом терминале: ./build/sitl/bin/arduplane --model plane -I0 --speedup 10
    2. Во втором терминале: python3 esn_optimizer.py 10
"""

import time
import csv
import math
import itertools
import sys
from datetime import datetime

from pymavlink import mavutil


# ───────────────────────────────────────────────────────────────────────────
# Конфигурация
# ───────────────────────────────────────────────────────────────────────────

SITL_CONNECTION = 'tcp:127.0.0.1:5762'
SPEEDUP = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0


def sitl_sleep(seconds):
    """Задержка в модельных секундах (делится на SPEEDUP)."""
    time.sleep(seconds / SPEEDUP)


# Индексы режимов ArduPlane (Mode::Number)
MODE_MANUAL   = 0
MODE_ACRO     = 4      # ← режим ACRO для rate-теста
MODE_FBWA     = 5
MODE_AUTO     = 10
MODE_RTL      = 11
MODE_LOITER   = 12
MODE_TAKEOFF  = 13
MODE_GUIDED   = 15

FORCE_ARM_MAGIC = 21196

# Газ
THROTTLE_CLIMB  = 1900
THROTTLE_ACRO   = 1800

# ───────────────────────────────────────────────────────────────────────────
# Гиперпараметры ESN
# ───────────────────────────────────────────────────────────────────────────

hyperparam_grid = {
    'RLL_ESN_SR':     [0.8, 0.9, 0.95],
    'RLL_ESN_LR':     [0.05, 0.1, 0.2],
    'RLL_ESN_LAMBDA': [0.99, 0.995, 0.999],
}

# Плохие PID — P в 2–3 раза выше оптимального, но НЕ в клиппинге
BAD_PID = {
    'RLL_RATE_P':    0.3,
    'PTCH_RATE_P':   0.2,
}

# Хорошие PID (для сброса между итерациями)
GOOD_PID = {
    'RLL_RATE_P':    0.08,
    'PTCH_RATE_P':   0.04,
    'RLL_RATE_I':    0.15,
    'PTCH_RATE_I':   0.15,
    'RLL_RATE_D':    0.0,
    'PTCH_RATE_D':   0.0,
    'RLL_RATE_SMAX': 150.0,
    'PTCH_RATE_SMAX': 150.0,
}

# ─── Параметры, отключающие маскирующие механизмы ───
ACRO_TEST_PARAMS = {
    # I = 0, чтобы интегратор не гасил раскачку
    'RLL_RATE_I':      0.0,
    'PTCH_RATE_I':     0.0,
    'RLL_RATE_IMAX':   0.0,
    'PTCH_RATE_IMAX':  0.0,

    # D и FF = 0, чтобы не демпфировали
    'RLL_RATE_D':      0.0,
    'PTCH_RATE_D':     0.0,
    'RLL_RATE_FF':     0.0,
    'PTCH_RATE_FF':    0.0,
    'RLL_RATE_D_FF':   0.0,
    'PTCH_RATE_D_FF':  0.0,
    'RLL_RATE_PDMX':   0.0,
    'PTCH_RATE_PDMX':  0.0,

    # Фильтры = 0, чтобы шум проходил напрямую
    'RLL_RATE_FLTD':   0.0,
    'RLL_RATE_FLTE':   0.0,
    'RLL_RATE_FLTT':   0.0,
    'PTCH_RATE_FLTD':  0.0,
    'PTCH_RATE_FLTE':  0.0,
    'PTCH_RATE_FLTT':  0.0,

    # SMAX = 0 (внутренний slew limiter всё равно остаётся, но параметр ослабляем)
    'RLL_RATE_SMAX':   0.0,
    'PTCH_RATE_SMAX':  0.0,

    # RMAX внешнего контура = 0 (для ACRO не критично, но полезно)
    'RLL2SRV_RMAX':       0.0,
    'PTCH2SRV_RMAX_UP':   0.0,
    'PTCH2SRV_RMAX_DN':   0.0,

    # Максимальные угловые скорости для ACRO — чтобы стик давал большие команды
    'ACRO_ROLL_RATE':   180.0,
    'ACRO_PITCH_RATE':  90.0,
    'ACRO_Y_RATE':      90.0,
    'ACRO_TRAINER':     0,      # без ограничений
    'ACRO_RP_EXPO':     0.0,    # линейный отклик

    'SR2_EXTRA1': 100.0,   # ATTITUDE на 100 Гц
}


# ───────────────────────────────────────────────────────────────────────────
# Обёртки над MAVLink
# ───────────────────────────────────────────────────────────────────────────

class Drone:
    def __init__(self, connection_string):
        self.master = mavutil.mavlink_connection(connection_string)
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat(timeout=30)
        print(f"Heartbeat from system {self.master.target_system}, "
              f"component {self.master.target_component}")
        self.master.recv_match(type='HEARTBEAT', blocking=False)

    def set_param(self, name, value):
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            name.encode('utf-8'),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=False)
            if msg is not None and msg.param_id.rstrip('\x00') == name:
                return
            time.sleep(0.01)

    def set_mode(self, mode_id, timeout=3.0):
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg is not None and msg.custom_mode == mode_id:
                return True
            time.sleep(0.05)
        return False

    def force_arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, FORCE_ARM_MAGIC, 0, 0, 0, 0, 0
        )
        deadline = time.time() + 3.0
        while time.time() < deadline:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=False)
            if msg is not None:
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        return True
                    print(f"  [WARN] Force-arm result: {msg.result}")
                    return False
            time.sleep(0.05)
        return False

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def rc_override(self, channels: dict):
        vals = [65535] * 8
        for ch, pwm in channels.items():
            if 1 <= ch <= 8:
                vals[ch-1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component, *vals
        )

    def release_rc_override(self):
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def drain(self):
        while self.master.recv_match(blocking=False) is not None:
            pass

    def get_attitude(self):
        msg = self.master.recv_match(type='ATTITUDE', blocking=False)
        if msg is None:
            return None
        return (math.degrees(msg.roll), math.degrees(msg.pitch))


# ───────────────────────────────────────────────────────────────────────────
# Вспомогательные процедуры
# ───────────────────────────────────────────────────────────────────────────

def set_params_batch(drone, params_dict, timeout=2.0):
    """
    Отправляет все параметры разом и ждёт ACK одним общим таймаутом.
    Экономит время при большом количестве параметров.
    """
    expected = set(params_dict.keys())
    confirmed = set()

    # 1. Отправить все параметры сразу
    for name, value in params_dict.items():
        drone.master.mav.param_set_send(
            drone.master.target_system,
            drone.master.target_component,
            name.encode('utf-8'),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    # 2. Собрать все ACK за один общий дедлайн
    deadline = time.time() + timeout
    while time.time() < deadline and confirmed != expected:
        msg = drone.master.recv_match(type='PARAM_VALUE', blocking=False)
        if msg is not None:
            pid = msg.param_id.rstrip('\x00')
            if pid in expected:
                confirmed.add(pid)
        time.sleep(0.001)

    missing = expected - confirmed
    if missing:
        print(f"    [WARN] Params not ACKed: {missing}")
    return len(confirmed) == len(expected)


def apply_hyperparams(drone, params):
    p = {**params, 'RLL_ESN_RESET': 1, 'PTCH_ESN_RESET': 1}
    set_params_batch(drone, p, timeout=2.0)
    sitl_sleep(1.0)


def prepare_acro_test(drone):
    """Устанавливает все параметры, чтобы сделать раскачку наблюдаемой."""
    print("  [Prep] Applying ACRO test parameters...")
    set_params_batch(drone, ACRO_TEST_PARAMS)
    sitl_sleep(0.3)


def apply_bad_pid(drone):
    set_params_batch(drone, BAD_PID)
    sitl_sleep(0.3)


def restore_good_pid(drone):
    set_params_batch(drone, GOOD_PID)
    sitl_sleep(0.3)


def collect_attitude_for(drone, duration_s, roll_samples, pitch_samples):
    """Собирает ATTITUDE за duration_s модельных секунд."""
    t_end = time.time() + duration_s / SPEEDUP
    while time.time() < t_end:
        att = drone.get_attitude()
        if att is not None:
            roll_samples.append(att[0])
            pitch_samples.append(att[1])
        #time.sleep(0.005)


# ───────────────────────────────────────────────────────────────────────────
# Тестовый полёт в ACRO
# ───────────────────────────────────────────────────────────────────────────

def run_test_flight(drone):
    """
    Тест в ACRO: резкие знакопеременные команды по крену и тангажу.
    """
    print("  [Flight] Setting TAKEOFF mode and force-arm...")
    drone.set_mode(MODE_TAKEOFF)
    sitl_sleep(0.5)

    if not drone.force_arm():
        raise RuntimeError("Force-arm failed")

    print("  [Flight] Climbing in TAKEOFF...")
    sitl_sleep(15)

    print("  [Flight] Switching to ACRO...")
    drone.set_mode(MODE_ACRO)
    sitl_sleep(1)

    drone.drain()

    # Газ
    print("  [Flight] Spinning up...")
    drone.rc_override({3: THROTTLE_CLIMB})
    sitl_sleep(2)

    roll_samples = []
    pitch_samples = []

    # ─── Крен: 3 цикла резких перебросов ───
    print("  [Flight] Roll oscillations...")
    drone.rc_override({3: THROTTLE_ACRO, 1: 1500, 2: 1500})
    sitl_sleep(0.5)

    for i in range(3):
        # Вправо
        drone.rc_override({3: THROTTLE_ACRO, 1: 1600, 2: 1500})
        collect_attitude_for(drone, 2.0, roll_samples, pitch_samples)
        # Влево
        drone.rc_override({3: THROTTLE_ACRO, 1: 1400, 2: 1500})
        collect_attitude_for(drone, 2.0, roll_samples, pitch_samples)

    # Центр
    drone.rc_override({3: THROTTLE_ACRO, 1: 1500, 2: 1500})
    sitl_sleep(1.5)

    # ─── Тангаж: 3 цикла ───
    print("  [Flight] Pitch oscillations...")
    for i in range(3):
        # Вверх
        drone.rc_override({3: THROTTLE_ACRO, 1: 1500, 2: 1650})
        collect_attitude_for(drone, 0.8, roll_samples, pitch_samples)
        # Вниз
        drone.rc_override({3: THROTTLE_ACRO, 1: 1500, 2: 1350})
        collect_attitude_for(drone, 0.8, roll_samples, pitch_samples)

    # Центр
    drone.rc_override({3: THROTTLE_ACRO, 1: 1500, 2: 1500})
    sitl_sleep(1)

    # ─── Метрики ───
    def stats(arr):
        if len(arr) < 10:
            return {'std': float('nan'), 'max': float('nan'), 'range': float('nan')}
        mean = sum(arr) / len(arr)
        var = sum((x - mean) ** 2 for x in arr) / len(arr)
        return {
            'std':   math.sqrt(var),
            'max':   max(abs(x) for x in arr),
            'range': max(arr) - min(arr),
        }

    roll_stat  = stats(roll_samples)
    pitch_stat = stats(pitch_samples)

    print("  [Flight] Returning to launch...")
    drone.release_rc_override()
    drone.set_mode(MODE_RTL)
    sitl_sleep(20)

    return {
        'roll_std_deg':      roll_stat['std'],
        'roll_max_abs_deg':  roll_stat['max'],
        'roll_range_deg':    roll_stat['range'],
        'pitch_std_deg':     pitch_stat['std'],
        'pitch_max_abs_deg': pitch_stat['max'],
        'pitch_range_deg':   pitch_stat['range'],
        'samples':           len(roll_samples),
    }


# ───────────────────────────────────────────────────────────────────────────
# Основной цикл
# ───────────────────────────────────────────────────────────────────────────

def main():
    out_csv = f"esn_grid_search_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    keys = list(hyperparam_grid.keys())
    values = list(hyperparam_grid.values())
    combinations = list(itertools.product(*values))

    print(f"SPEEDUP = {SPEEDUP}")
    print(f"Total combinations: {len(combinations)}")
    print(f"Output CSV: {out_csv}\n")

    results = []
    drone = Drone(SITL_CONNECTION)

    # Один раз настроить ACRO-параметры
    prepare_acro_test(drone)

    try:
        for idx, combo in enumerate(combinations):
            params = dict(zip(keys, combo))
            print(f"[{idx+1}/{len(combinations)}] Testing: {params}")

            try:
                restore_good_pid(drone)
                prepare_acro_test(drone)   # повторно, на случай если reset сбросил
                apply_bad_pid(drone)
                apply_hyperparams(drone, params)
                metrics = run_test_flight(drone)

                row = {**params, **metrics,
                       'timestamp': datetime.now().isoformat()}
                results.append(row)

                with open(out_csv, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=row.keys())
                    writer.writeheader()
                    writer.writerows(results)

                print(f"  Metrics: roll_std={metrics['roll_std_deg']:.2f}, "
                      f"pitch_std={metrics['pitch_std_deg']:.2f}, "
                      f"samples={metrics['samples']}")

            except Exception as e:
                print(f"  ERROR: {e}")
                try:
                    drone.release_rc_override()
                    drone.set_mode(MODE_RTL)
                    sitl_sleep(20)
                except Exception:
                    pass
                continue

    finally:
        drone.disarm()
        print(f"\nGrid search complete. Results saved to {out_csv}")

    if results:
        valid = [r for r in results if not math.isnan(r.get('roll_std_deg', float('nan')))]
        if valid:
            best = min(valid, key=lambda r: r['roll_std_deg'])
            print("\nBest trial (min roll_std):")
            for k, v in best.items():
                print(f"  {k} = {v}")
        else:
            print("\nNo valid results (all NaN).")


if __name__ == "__main__":
    main()