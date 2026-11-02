"""Static Qt Widgets launcher. The headless QProcess supervisor owns script handoff."""
import argparse
import configparser
import json
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(sys.executable).parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parent
# PyInstaller onedir places the executable beside config/media; all writes are local.
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
RUNTIME = ROOT / '_tmp'
RUNTIME.mkdir(exist_ok=True)
for name in ('TEMP', 'TMP', 'TMPDIR', 'XDG_CACHE_HOME'):
    os.environ[name] = str(RUNTIME)
if os.name == 'nt':
    os.environ.setdefault('QT_MULTIMEDIA_PREFERRED_PLUGINS', 'windowsmediafoundation')
from PyQt5 import QtCore as C, QtGui as G, QtWidgets as W
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget


def settings(path):
    cfg = configparser.ConfigParser(interpolation=None)
    with Path(path).open(encoding='utf-8-sig') as f:
        cfg.read_file(f)
    keys = [k.strip() for k in cfg['launcher']['order'].split(',') if k.strip()]
    if not keys or len(keys) != len(set(keys)) or cfg['launcher']['runMode'] not in ('video', 'script'):
        raise ValueError('launcher.ini: invalid order or runMode')
    for k in keys:
        for required in ('title', 'description', 'color', 'video', 'script', 'env'):
            if not cfg[k].get(required):
                raise ValueError(f'launcher.ini: {k}.{required} missing')
    return cfg, keys


def resolve(value, base):
    p = Path(value)
    return p if p.is_absolute() else base / p


class Home(W.QWidget):
    def __init__(self, cfg, keys, base, clicked):
        super().__init__()
        self.background = G.QPixmap(str(resolve(cfg['launcher']['background'], base)))
        if self.background.isNull():
            raise ValueError('主页静态底图不存在')
        self.buttons = []
        for i, key in enumerate(keys):
            m = cfg[key]
            b = W.QPushButton('', self)
            b.setAccessibleName(m['title'] + '模式')
            b.setStyleSheet('QPushButton {background:transparent;border:0;}'
                            'QPushButton:focus {border:3px solid #6940c9;border-radius:22px;}')
            b.clicked.connect(lambda checked=False, k=key: clicked(k))
            self.buttons.append(b)
        self.error = W.QLabel(os.environ.get('ADAS_LAUNCHER_LAST_ERROR', ''), self)
        self.error.setStyleSheet('color:#a32035;font-size:22px;background:transparent;')
        self.exit_button = W.QPushButton('退出软件', self)
        self.exit_button.setStyleSheet('background:#faf8ff;color:#5b35ad;border:2px solid #a58be5;'
                                      'border-radius:14px;font:24px "Microsoft YaHei";')
        self.exit_button.clicked.connect(lambda: self.window().close())

    def resizeEvent(self, event):
        scale = min(self.width()/1920, self.height()/1080)
        ox = (self.width()-1920*scale)/2
        oy = (self.height()-1080*scale)/2
        for i, b in enumerate(self.buttons):
            b.setGeometry(int(ox+(100+i*440)*scale), int(oy+420*scale), int(400*scale), int(340*scale))
        self.error.setGeometry(int(ox+100*scale), int(oy+790*scale), int(1720*scale), int(48*scale))
        self.exit_button.setGeometry(int(ox+1660*scale), int(oy+270*scale), int(168*scale), int(58*scale))
        super().resizeEvent(event)

    def paintEvent(self, event):
        p = G.QPainter(self)
        p.fillRect(self.rect(), G.QColor('#f4f1ff'))
        scale = min(self.width()/1920, self.height()/1080)
        p.translate((self.width()-1920*scale)/2, (self.height()-1080*scale)/2)
        p.scale(scale, scale)
        p.drawPixmap(0, 0, self.background)
        p.fillRect(40, 41, 1840, 184, G.QColor('#ffffff'))
        for i in range(4):
            p.fillRect(110+i*440, 674, 380, 65, G.QColor('#faf8ff'))


class Window(W.QMainWindow):
    def __init__(self, args, cfg, keys):
        super().__init__()
        self.args, self.cfg, self.keys = args, cfg, keys
        self.base = Path(args.config).resolve().parent
        self.setWindowTitle('ADAS 四模式演示')
        self.resize(1920, 1080)
        self.stack = W.QStackedWidget()
        self.setCentralWidget(self.stack)
        self.home = Home(cfg, keys, self.base, self.open_mode)
        self.stack.addWidget(self.home)
        self.page = W.QWidget()
        self.video = QVideoWidget(self.page)
        self.back = W.QPushButton('‹ 返回', self.page)
        self.back.setStyleSheet('background:#35215e;color:white;font-size:27px;border:0;')
        self.back.clicked.connect(self.go_home)
        self.stack.addWidget(self.page)
        self.player = QMediaPlayer(self)
        self.player.setVideoOutput(self.video)
        self.player.mediaStatusChanged.connect(self.media_status)
        self.player.error.connect(self.media_error)
        self.active = None
        self.media_alias = None
        self.capture_index = -1
        self.capture_wait = 0
        self.capture_rows = []
        if args.windowed:
            self.show()
        else:
            self.showFullScreen()
        self.back.raise_()
        if args.capture_dir:
            Path(args.capture_dir).mkdir(parents=True, exist_ok=True)
            C.QTimer.singleShot(900, self.capture_home)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'video'):
            self.video.setGeometry(self.page.rect())
            self.back.setGeometry(320, 28, 144, 58)
            self.back.raise_()

    def open_mode(self, key):
        if self.active is not None:
            return
        if self.cfg['launcher']['runMode'] == 'script':
            request = os.environ.get('ADAS_LAUNCHER_REQUEST')
            if not request:
                self.home.error.setText('请通过启动器监督进程进入脚本模式。')
                return
            Path(request).write_text(key, encoding='utf-8')
            W.QApplication.instance().exit(75)
            return
        path = resolve(self.cfg[key]['video'], self.base)
        if not path.is_file():
            self.home.error.setText('视频不存在：' + str(path))
            return
        self.active = key
        self.stack.setCurrentWidget(self.page)
        self.video.setGeometry(self.page.rect())
        self.back.setGeometry(320, 28, 144, 58)
        self.back.raise_()
        if os.name == 'nt':
            # Qt5 WMF rejects some Unicode URLs; an ASCII hardlink keeps bytes untouched.
            self.media_alias = RUNTIME/f'video_{self.keys.index(key)}.mp4'
            self.media_alias.unlink(missing_ok=True)
            os.link(path, self.media_alias)
            path = self.media_alias
        self.player.setMedia(QMediaContent(C.QUrl.fromLocalFile(str(path))))
        self.player.play()

    def go_home(self):
        self.player.stop()
        self.active = None
        self.stack.setCurrentWidget(self.home)
        self.home.buttons[0].setFocus()

    def media_status(self, status):
        if status == QMediaPlayer.EndOfMedia:
            self.go_home()

    def media_error(self, error):
        if error == QMediaPlayer.NoError:
            return
        self.home.error.setText(self.player.errorString())
        if self.args.capture_dir:
            print('Playback error:', int(error), repr(self.player.errorString()), flush=True)
            W.QApplication.instance().exit(4)
        else:
            self.go_home()

    def keyPressEvent(self, event):
        if event.key() in (C.Qt.Key_Escape, C.Qt.Key_Back):
            self.go_home() if self.active else self.close()
        elif not self.active and C.Qt.Key_1 <= event.key() < C.Qt.Key_1+len(self.keys):
            self.open_mode(self.keys[event.key()-C.Qt.Key_1])
        else:
            super().keyPressEvent(event)

    def capture_home(self):
        self.grab().save(str(Path(self.args.capture_dir)/'01_home.png'))
        self.capture_next()

    def capture_next(self):
        self.capture_index += 1
        modes = ['cruise', 'surround', 'intersection', 'nav']
        if self.capture_index >= len(modes):
            (Path(self.args.capture_dir)/'GUI_SMOKE.json').write_text(json.dumps({
                'passed': True, 'qt_version': C.QT_VERSION_STR, 'ui': 'Qt Widgets',
                'capture': 'actual QWidget window', 'rows': self.capture_rows,
                'hardware_RK3568_tested': False}, ensure_ascii=False, indent=2), encoding='utf-8')
            W.QApplication.instance().quit()
            return
        self.open_mode(modes[self.capture_index])
        self.capture_wait = time.monotonic()
        C.QTimer.singleShot(200, self.capture_playback)

    def capture_playback(self):
        if time.monotonic()-self.capture_wait > 25:
            W.QApplication.instance().exit(5)
            return
        if self.player.position() < (4000 if self.active == 'cruise' else 2500):
            C.QTimer.singleShot(200, self.capture_playback)
            return
        path = Path(self.args.capture_dir)/f'{self.capture_index+2:02}_{self.active}_playback.png'
        # QWidget native video child is included by the platform screen grab.
        screen = W.QApplication.primaryScreen()
        if not screen.grabWindow(self.winId()).save(str(path)):
            W.QApplication.instance().exit(6)
            return
        self.capture_rows.append({'key': self.active, 'position_ms': self.player.position(), 'file': path.name})
        self.go_home()
        C.QTimer.singleShot(250, self.capture_next)


class Supervisor(C.QObject):
    def __init__(self, app, args, cfg, keys):
        super().__init__()
        self.app, self.args, self.cfg, self.keys = app, args, cfg, keys
        self.base = Path(args.config).resolve().parent
        self.request = RUNTIME / f'request_{os.getpid()}.txt'
        self.stopfile = RUNTIME / 'return.request'
        self.stopfile.unlink(missing_ok=True)
        self.proc = C.QProcess(self)
        self.proc.setProcessChannelMode(C.QProcess.ForwardedChannels)
        self.proc.finished.connect(self.finished)
        self.proc.errorOccurred.connect(self.error)
        self.exit_input = C.QProcess(self)
        self.exit_input.finished.connect(self.exit_input_finished)
        self.running_script = False
        self.return_requested = False
        self.closing = False
        self.error_message = ''
        self.timer = C.QTimer(self)
        self.timer.timeout.connect(self.check_return)
        self.timer.start(150)
        self.stdin_notifier = None
        self.saved_tty = None
        if os.name != 'nt' and sys.stdin.isatty():
            import termios, tty
            self.saved_tty = termios.tcgetattr(0)
            tty.setcbreak(0)
            self.stdin_notifier = C.QSocketNotifier(0, C.QSocketNotifier.Read, self)
            self.stdin_notifier.activated.connect(self.keyboard)
        app.aboutToQuit.connect(self.cleanup)
        self.start_gui()

    def env(self):
        e = C.QProcessEnvironment.systemEnvironment()
        e.insert('ADAS_LAUNCHER_REQUEST', str(self.request))
        e.insert('ADAS_LAUNCHER_LAST_ERROR', self.error_message)
        return e

    def start_gui(self):
        self.running_script = False
        self.return_requested = False
        self.close_exit_input()
        self.request.unlink(missing_ok=True)
        self.proc.setProcessEnvironment(self.env())
        cmd = [sys.executable] if getattr(sys, 'frozen', False) else [sys.executable, str(Path(__file__).resolve())]
        cmd += ['--gui', '--config', self.args.config]
        if self.args.run_mode:
            cmd += ['--run-mode', self.args.run_mode]
        if self.args.windowed:
            cmd += ['--windowed']
        self.proc.start(cmd[0], cmd[1:])

    def finished(self, code, status):
        if self.closing:
            return
        if self.running_script:
            self.error_message = '' if code == 0 else f'C 程序退出码 {code}，请检查终端日志。'
            self.start_gui()
        elif code == 75:
            key = self.request.read_text(encoding='utf-8') if self.request.exists() else ''
            if key not in self.keys:
                self.app.exit(2); return
            m = self.cfg[key]
            script = resolve(m['script'], self.base)
            if not script.is_file() or os.name == 'nt':
                self.error_message = '脚本模式需在 RK/Linux 运行，并配置实际 C 程序路径。'
                self.start_gui(); return
            e = self.env()
            for pair in m['env'].split(';'):
                name, value = pair.split('=', 1)
                e.insert(name.strip(), value.strip())
            e.insert('ADAS_INPUT', m.get('input', 'device'))
            e.insert('ADAS_EXTRA_ARGS', m.get('args', ''))
            if not self.open_exit_input():
                self.error_message = '返回按钮和 Esc 未就绪，请检查显示配置及 return_input。'
                self.start_gui(); return
            e.insert('ADAS_LAUNCHER_RETURN_UI', '1')
            self.proc.setProcessEnvironment(e)
            self.proc.setWorkingDirectory(str(self.base))
            self.running_script = True
            # A private process group lets Return stop only this launched C subtree.
            self.proc.start('/usr/bin/setsid', ['/bin/sh', str(script)])
        else:
            self.app.exit(code)

    def error(self, error):
        if error == C.QProcess.FailedToStart:
            print(self.proc.errorString(), file=sys.stderr)
            self.app.exit(3)

    def open_exit_input(self):
        helper = ROOT/'return_input'
        if not sys.platform.startswith('linux') or not helper.is_file():
            return False
        self.exit_input.start(str(helper), [])
        if not self.exit_input.waitForStarted(1000):
            return False
        if not self.exit_input.waitForReadyRead(1000):
            return False
        if bytes(self.exit_input.readAllStandardOutput()).strip() != b'READY':
            return False
        self.exit_input.readyReadStandardOutput.connect(self.exit_input_ready)
        return True

    def exit_input_ready(self):
        lines = bytes(self.exit_input.readAllStandardOutput()).splitlines()
        if any(line in (b'RETURN_BUTTON', b'RETURN_ESC') for line in lines):
            print('RETURN_INPUT ' + lines[-1].decode(), flush=True)
            self.stop_script()

    def exit_input_finished(self, code, status):
        if self.running_script and not self.closing and not self.return_requested:
            self.error_message = '返回输入监听器退出，已停止当前处理。'
            self.stop_script()

    def close_exit_input(self):
        if self.exit_input.state() != C.QProcess.NotRunning:
            self.exit_input.terminate()
            if not self.exit_input.waitForFinished(500):
                self.exit_input.kill(); self.exit_input.waitForFinished(500)
        try:
            self.exit_input.readyReadStandardOutput.disconnect(self.exit_input_ready)
        except TypeError:
            pass

    def keyboard(self, fd):
        if b'\x1b' in os.read(0, 64) and self.running_script:
            self.stop_script()

    def check_return(self):
        if self.stopfile.exists():
            self.stopfile.unlink()
            if self.running_script:
                self.stop_script()

    def stop_script(self):
        pid = int(self.proc.processId())
        if pid and self.running_script and not self.return_requested:
            self.return_requested = True
            try:
                # C owns its display child; let it drain DMA and close the pipe first.
                exe = Path(f'/proc/{pid}/exe')
                if sys.platform.startswith('linux') and exe.resolve().name == 'play_adas_yolo':
                    os.kill(pid, signal.SIGTERM)
                else:
                    os.killpg(pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            C.QTimer.singleShot(6000, lambda: self.kill_if_same(pid))

    def kill_if_same(self, pid):
        if self.running_script and self.proc.processId() == pid:
            try:
                os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def cleanup(self):
        self.closing = True
        self.close_exit_input()
        if self.running_script:
            self.stop_script()
            if not self.proc.waitForFinished(6000):
                self.kill_if_same(int(self.proc.processId()))
                self.proc.waitForFinished(1000)
        elif self.proc.state() != C.QProcess.NotRunning:
            self.proc.terminate()
            if not self.proc.waitForFinished(2000):
                self.proc.kill()
                self.proc.waitForFinished(1000)
        if self.saved_tty:
            import termios
            termios.tcsetattr(0, termios.TCSADRAIN, self.saved_tty)
        self.request.unlink(missing_ok=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', default=str(ROOT/'launcher.ini'))
    p.add_argument('--windowed', action='store_true')
    p.add_argument('--gui', action='store_true')
    p.add_argument('--run-mode', choices=['video', 'script'])
    p.add_argument('--capture-dir')
    p.add_argument('--return', dest='return_home', action='store_true')
    a = p.parse_args()
    if a.return_home:
        (RUNTIME/'return.request').touch(); return 0
    cfg, keys = settings(a.config)
    if a.run_mode:
        cfg['launcher']['runMode'] = a.run_mode
    if a.gui or a.capture_dir:
        app = W.QApplication(sys.argv[:1])
        app.setFont(G.QFont('Microsoft YaHei', 12))
        window = Window(a, cfg, keys)
        return app.exec_()
    app = C.QCoreApplication(sys.argv[:1])
    lock = C.QLockFile(str(RUNTIME/'launcher.lock'))
    if not lock.tryLock(0):
        print('启动器已在运行。', file=sys.stderr)
        return 2
    controller = Supervisor(app, a, cfg, keys)
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
