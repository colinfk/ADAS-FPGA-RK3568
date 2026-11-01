#!/usr/bin/env python3
"""Supervise a desktop shortcut and the existing X11 return listener."""
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
HELPER = ROOT / 'launcher' / 'return_input'
stopping = 0


def on_signal(signum, _frame):
    global stopping
    stopping = signum


def stop_player(player):
    if player.poll() is not None:
        return player.returncode
    try:
        name = Path(f'/proc/{player.pid}/exe').resolve().name
        if name in ('play_adas_yolo', 'play_adas_yolo_aarch64'):
            os.kill(player.pid, signal.SIGTERM)
        else:
            os.killpg(player.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        return player.wait(timeout=6)
    except subprocess.TimeoutExpired:
        print('RETURN_CLEANUP_TIMEOUT: terminating this shortcut process group', file=sys.stderr, flush=True)
        try:
            os.killpg(player.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        player.wait()
        return 1


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ('passthrough', 'yolo_only'):
        print('Expected passthrough or yolo_only', file=sys.stderr)
        return 2
    os.environ.setdefault('DISPLAY', ':0')
    authority = Path('/var/run/lightdm/root/:0')
    if not os.environ.get('XAUTHORITY') and authority.is_file():
        os.environ['XAUTHORITY'] = str(authority)
    if not os.access(HELPER, os.X_OK):
        tmp = HELPER.with_name(f'.return_input.{os.getpid()}')
        try:
            subprocess.run(['cc', '-O2', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I', str(ROOT/'rk3568/npu_runtime'),
                            str(ROOT/'launcher/tools/return_input.c'), '-lX11', '-lXi',
                            '-o', str(tmp)], check=True)
            os.replace(tmp, HELPER)
        except (OSError, subprocess.CalledProcessError) as exc:
            print(f'RETURN_INPUT_UNAVAILABLE: {exc}', file=sys.stderr, flush=True)
            return 1
        finally:
            tmp.unlink(missing_ok=True)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    listener = None
    player = None
    selector = selectors.DefaultSelector()
    buffer = b''
    try:
        listener = subprocess.Popen([str(HELPER)], stdout=subprocess.PIPE, bufsize=0)
        selector.register(listener.stdout, selectors.EVENT_READ)
        deadline = time.monotonic()+3
        ready = False
        while not ready and not stopping and time.monotonic()<deadline:
            if not selector.select(.1):
                continue
            chunk = os.read(listener.stdout.fileno(), 4096)
            if not chunk:
                break
            buffer += chunk
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                if line.strip() == b'READY':
                    ready = True
                elif line.strip() in (b'RETURN_BUTTON', b'RETURN_ESC'):
                    print('RETURN_INPUT before playback; ignoring', flush=True)
        if not ready or listener.poll() is not None:
            print('RETURN_INPUT_UNAVAILABLE: click listener did not become ready', file=sys.stderr, flush=True)
            return 1
        player = subprocess.Popen(['/bin/sh', str(ROOT/'start.sh'), sys.argv[1]], start_new_session=True)
        print('RETURN_INPUT_READY: 点击屏幕或按 Esc 可退出', flush=True)
        while not stopping:
            rc = player.poll()
            if rc is not None:
                return rc if rc >= 0 else 128-rc
            events = selector.select(.1)
            if events:
                chunk = os.read(listener.stdout.fileno(), 4096)
                if not chunk:
                    print('RETURN_INPUT_FAILED: listener exited; stopping shortcut', file=sys.stderr, flush=True)
                    stop_player(player)
                    return 1
                buffer += chunk
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    if line.strip() in (b'RETURN_BUTTON', b'RETURN_ESC'):
                        print('RETURN_INPUT '+line.strip().decode('ascii'), flush=True)
                        rc = stop_player(player)
                        return 0 if rc in (0, -signal.SIGTERM) else 1
            elif listener.poll() is not None:
                print('RETURN_INPUT_FAILED: listener exited; stopping shortcut', file=sys.stderr, flush=True)
                stop_player(player)
                return 1
        if player is not None:
            stop_player(player)
        return 128+stopping
    except OSError as exc:
        print(f'SHORTCUT_START_FAILED: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        if player is not None and player.poll() is None:
            stop_player(player)
        selector.close()
        if listener is not None:
            if listener.poll() is None:
                listener.terminate()
                try:
                    listener.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    listener.kill()
                    listener.wait()
            if listener.stdout is not None:
                listener.stdout.close()


if __name__ == '__main__':
    sys.exit(main())
