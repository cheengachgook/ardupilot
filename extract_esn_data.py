#!/usr/bin/env python3
"""
extract_esn_data.py
Извлекает данные ESN из логов ArduPilot, синхронизируя 12 признаков из разных сообщений.
"""

import os
import argparse
import csv
from pymavlink import mavutil

def find_last_logs(logs_dir, n):
    lastlog_path = os.path.join(logs_dir, 'LASTLOG.TXT')
    if not os.path.exists(lastlog_path):
        raise FileNotFoundError(f"LASTLOG.TXT not found in {logs_dir}")
    with open(lastlog_path, 'r') as f:
        last_num = int(f.read().strip())
    logs = []
    for i in range(n):
        num = last_num - i
        if num < 1:
            break
        fname = f"{num:08d}.BIN"
        fpath = os.path.join(logs_dir, fname)
        if os.path.exists(fpath):
            logs.append(fpath)
    return logs

def extract_full_features(log_path, step_sec):
    mlog = mavutil.mavlink_connection(log_path)
    
    # Словари для хранения последних значений по времени
    data_att = {}
    data_arsp = {}
    data_pos = {}
    data_pidr = {}
    data_esn = {}
    
    print(f"Parsing {log_path}...")
    while True:
        msg = mlog.recv_match(type=['ATT', 'ARSP', 'POS', 'PIDR', 'ESN'])
        if msg is None:
            break
        t = msg.TimeUS
        mtype = msg.get_type()
        
        if mtype == 'ATT':
            data_att[t] = msg
        elif mtype == 'ARSP':
            data_arsp[t] = msg
        elif mtype == 'POS':
            data_pos[t] = msg
        elif mtype == 'PIDR':
            data_pidr[t] = msg
        elif mtype == 'ESN' and msg.Ax == 0: # Только Roll
            data_esn[t] = msg

    mlog.close()

    # Теперь проходим по ESN сообщениям (они самые редкие, 25 Гц) и собираем 12 признаков
    samples = []
    last_time = None
    step_us = int(step_sec * 1e6)
    
    # Вспомогательная функция для поиска ближайшего сообщения по времени
    def get_closest(data_dict, target_time):
        if not data_dict: return None
        # Простой поиск (так как данные идут по порядку)
        closest_t = min(data_dict.keys(), key=lambda t: abs(t - target_time))
        # Разрешаем разницу во времени не более 0.1 сек (100000 us)
        if abs(closest_t - target_time) < 100000:
            return data_dict[closest_t]
        return None

    for t, esn_msg in sorted(data_esn.items()):
        if last_time is not None and (t - last_time) < step_us:
            continue
            
        att = get_closest(data_att, t)
        arsp = get_closest(data_arsp, t)
        pos = get_closest(data_pos, t)
        pidr = get_closest(data_pidr, t)
        
        # Если каких-то данных нет, подставляем безопасные значения по умолчанию
        # 1. roll_error_deg
        roll_err = (att.DesRoll - att.Roll) if att else 0.0
        # 7. pitch_error_deg
        pitch_err = (att.DesPitch - att.Pitch) if att else 0.0
        # 8. yaw_rate_deg (используем ATT.Yaw или производную, здесь берем 0 если нет)
        yaw_rate = att.Yaw if att else 0.0
        # 9. airspeed
        airspeed = arsp.Airspeed if arsp else 0.0
        # 10. alt
        alt = pos.Alt if pos else 0.0
        
        # 3, 4, 5, 6 из PIDR
        rate_err = pidr.Err if pidr else 0.0
        I_roll = pidr.I if pidr else 0.0
        target_rate = pidr.Tar if pidr else 0.0
        actual_rate = pidr.Act if pidr else 0.0
        
        # 2. d_roll_error (производная ошибки)
        # Считаем разницу с предыдущим сэмплом
        d_roll_err = 0.0
        if samples:
            prev_t, prev_feats = samples[-1]
            dt = (t - prev_t) / 1e6
            if dt > 0:
                d_roll_err = (roll_err - prev_feats[0]) / dt
        
        # 11, 12 из ESN
        scP = esn_msg.ScP
        bKP = esn_msg.bKP
        
        features = [
            roll_err,       # 1
            d_roll_err,     # 2
            rate_err,       # 3
            I_roll,         # 4
            target_rate,    # 5
            actual_rate,    # 6
            pitch_err,      # 7
            yaw_rate,       # 8
            airspeed,       # 9
            alt,            # 10
            scP,            # 11
            bKP             # 12
        ]
        
        samples.append((t, features))
        last_time = t

    return samples

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--logs_dir', default=os.path.expanduser('~/ardupilot/logs'))
    parser.add_argument('--n_epochs', type=int, default=1)
    parser.add_argument('--step_sec', type=float, default=0.5)
    parser.add_argument('--output', default='esn_pretrain.csv')
    args = parser.parse_args()

    logs = find_last_logs(args.logs_dir, args.n_epochs)
    if not logs:
        print("No log files found.")
        return

    all_samples = []
    for log in logs:
        samples = extract_full_features(log, args.step_sec)
        all_samples.extend(samples)

    if not all_samples:
        print("No valid ESN samples extracted.")
        return

    with open(args.output, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_us'] + [f'f{i}' for i in range(12)])
        for t, feats in all_samples:
            writer.writerow([t] + feats)

    print(f"Saved {len(all_samples)} samples to {args.output}")

if __name__ == '__main__':
    main()