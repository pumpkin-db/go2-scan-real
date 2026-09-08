#!/usr/bin/env python3
"""Pre-acquisition OK3588 system/PTP clock check. Never rewrite ROS stamps."""
import json
import os
import pathlib
import re
import shlex
import subprocess
import time

import paramiko


def main():
    # Credentials are machine-local and gitignored. Environment variables may
    # override the file, while host/user have deployment-safe defaults.
    go2_root = pathlib.Path(__file__).resolve().parents[3]
    config_path = pathlib.Path(os.environ.get(
        'GO2_BOARD_CONFIG', go2_root / 'config/local/board.json'))
    config = {}
    if config_path.is_file():
        config = json.loads(config_path.read_text())
    host = os.environ.get('GO2_BOARD_HOST', config.get('host', '192.168.123.30'))
    user = os.environ.get('GO2_BOARD_USER', config.get('user', 'root'))
    password = os.environ.get('GO2_BOARD_PASSWORD', config.get('password')) or None
    port = int(os.environ.get('GO2_BOARD_PORT', config.get('port', 22)))
    if subprocess.check_output(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'], text=True).strip() != 'yes':
        raise RuntimeError('NX NTP is not synchronized; refusing to distribute its clock')
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    # Same trust policy as the existing oksh.py connection helper.
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, port=port, username=user, password=password,
                   look_for_keys=True, allow_agent=True,
                   timeout=8, auth_timeout=8, banner_timeout=8)
    try:
        def run(command):
            _, stdout, stderr = client.exec_command(command, timeout=5)
            output, error = stdout.read().decode(), stderr.read().decode()
            if stdout.channel.recv_exit_status():
                raise RuntimeError('remote command failed: ' + error.strip())
            return output.strip()

        def measure():
            samples = []
            for _ in range(5):
                start = time.time()
                remote = float(run('date +%s.%N'))
                end = time.time()
                samples.append((end - start, (start + end) / 2 - remote))
            return min(samples)

        active_slam = run("ps -eo comm | grep -E '^(pointlio_mappin|fastlio_mapping)' || true")
        if active_slam:
            raise RuntimeError('OK3588 SLAM is active: ' + active_slam)
        board_driver = run("ps -eo comm | grep -E '^livox_ros_drive' || true")
        if board_driver:
            # The 3908 deployment intentionally keeps its driver process alive
            # after scanning stops.  Process existence is therefore not an
            # acquisition signal.  Refuse a clock step only when fresh LiDAR
            # messages are actually flowing on the board ROS master.
            live = run("bash -lc 'source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
                       "export ROS_MASTER_URI=http://127.0.0.1:11311; "
                       "timeout 2 rostopic echo -n1 --noarr /livox/lidar >/dev/null 2>&1 "
                       "&& echo ACTIVE || echo INACTIVE'")
            if live.strip() == 'ACTIVE':
                raise RuntimeError('OK3588 LiDAR data is active; stop scanning before clock synchronization')
            print('[CLOCK] OK3588 Livox driver is resident but not publishing; safe to synchronize', flush=True)
        processes = run('ps -eo args')
        if 'ptp4l -i eth0' not in processes or 'phc2sys -s CLOCK_REALTIME -c /dev/ptp0' not in processes:
            raise RuntimeError('expected OK3588 system -> PHC -> MID360 PTP chain is absent')
        rtt, correction = measure()
        print(f'[CLOCK] NX minus OK3588={correction:+.6f}s RTT={rtt:.6f}s', flush=True)
        if rtt > 0.1:
            raise RuntimeError('clock measurement RTT too large (>100ms); no clock change made')
        if abs(correction) > 0.02:
            code = f'import time; time.clock_settime(time.CLOCK_REALTIME, time.time() + {correction!r})'
            run('python3 -c ' + shlex.quote(code))
        # A large system-clock step is not necessarily stepped by an already
        # running phc2sys servo. Align PHC explicitly while acquisition is idle.
        run('/usr/local/sbin/phc_ctl /dev/ptp0 set')
        stable = 0
        for _ in range(30):
            output = run('/usr/local/sbin/phc_ctl /dev/ptp0 cmp')
            match = re.search(r'offset from CLOCK_REALTIME is\s+([-+\d]+)ns', output)
            if not match:
                raise RuntimeError('cannot parse PHC comparison: ' + output)
            phc_offset = int(match.group(1)) / 1e9
            if abs(phc_offset) < 0.02:
                stable += 1
                if stable >= 6:
                    break
            else:
                stable = 0
            time.sleep(0.5)
        else:
            raise RuntimeError(f'PTP hardware clock has not converged: {phc_offset:+.6f}s')
        rtt, residual = measure()
        if rtt > 0.1 or abs(residual) + rtt / 2 > 0.1:
            raise RuntimeError(f'clock verification failed: offset={residual:+.6f}s RTT={rtt:.6f}s')
        print(f'[CLOCK] verified OK3588 offset={residual:+.6f}s PHC offset={phc_offset:+.6f}s; ROS stamps unchanged', flush=True)
    finally:
        client.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        raise SystemExit('[CLOCK] FAILED: ' + str(error))
