import collections
import json
from pathlib import Path
import queue
import secrets
import socket
import struct
import threading
import time

from .protocol import Continuity, RATES, decode
from .dsp import Processor


class Recorder:
    """Length-prefixed S31Q v2 datagrams; bounded writer queue."""
    def __init__(self, path):
        self.path = Path(path)
        self.file = self.path.open('xb')
        self.file.write(b'S31IQ\x02\x00\x00')
        self.queue = queue.Queue(8192)
        self.drops = self.packets = self.bytes = 0
        self.error = ''
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def append(self, data):
        try:
            self.queue.put_nowait(data)
        except queue.Full:
            self.drops += 1

    def _write(self):
        try:
            while True:
                data = self.queue.get()
                if data is None:
                    break
                self.file.write(struct.pack('<H', len(data)))
                self.file.write(data)
                self.packets += 1
                self.bytes += len(data)
        except OSError as e:
            self.error = str(e)
        finally:
            self.file.close()

    def stop(self):
        while self.thread.is_alive():
            try:
                self.queue.put(None, timeout=.2)
                break
            except queue.Full:
                pass
        self.thread.join()
        info = self.snapshot()
        self.path.with_suffix('.json').write_text(json.dumps(info, indent=2), encoding='utf-8')
        return info

    def snapshot(self):
        return dict(path=str(self.path), packets=self.packets, bytes=self.bytes,
                    dropped_packets=self.drops, error=self.error,
                    protocol_version=2, payload_checksum='none')


class Receiver:
    def __init__(self, board='169.254.9.36', bind='169.254.9.35', port=9875, recording_dir='artifacts/recordings'):
        self.peer = (socket.gethostbyname(board), port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
        self.sock.bind((bind, 0))
        self.sock.settimeout(.1)
        self.receive_buffer = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.replies = queue.Queue()
        self.command_lock = threading.Lock()
        self.waiting_prefix = None
        self.lock = threading.Lock()
        self.dsp_lock = threading.Lock()
        self.dsp = Processor()
        self.chunks = collections.deque(maxlen=128)
        self.continuity = Continuity(0)
        self.device = {}
        self.running = False
        self.closed = threading.Event()
        self.started = self.last_packet = self.last_device = 0
        self.frame_number = 0
        self.error = ''
        self.settings = dict(rate=4_000_000, frequency=2412, gain=40)
        self.recorder = None
        self.last_recording = None
        self.recording_dir = Path(recording_dir)
        self.threads = [threading.Thread(target=f, daemon=True) for f in (self._receive, self._monitor, self._process)]
        for t in self.threads:
            t.start()

    def command(self, text, prefix='OK', timeout=3):
        with self.command_lock:
            while True:
                try:
                    self.replies.get_nowait()
                except queue.Empty:
                    break
            self.waiting_prefix = prefix
            try:
                self.sock.sendto((text + '\n').encode(), self.peer)
                until = time.monotonic() + timeout
                while time.monotonic() < until:
                    try:
                        reply = self.replies.get(timeout=max(.01, until-time.monotonic()))
                    except queue.Empty:
                        break
                    if reply.startswith('ERR'):
                        raise RuntimeError(reply)
                    if reply.startswith(prefix):
                        return reply
                raise TimeoutError(f'{text}: device did not reply')
            finally:
                self.waiting_prefix = None

    def start(self, rate=4_000_000, frequency=2412, gain=40):
        if rate not in RATES:
            raise ValueError('unsupported sample rate')
        if not 100 <= frequency <= 6000 or not 0 <= gain <= 127:
            raise ValueError('frequency or gain outside input range')
        if self.running:
            self.stop()
        self.command('NET?', 'NET ')
        self.command(f'FREQ {frequency}')
        self.command(f'GAIN MANUAL {gain}')
        session = secrets.randbits(32) or 1
        self.settings = dict(rate=rate, frequency=frequency, gain=gain)
        with self.lock:
            self.continuity = Continuity(session)
            self.chunks.clear()
            self.started = time.monotonic()
            self.last_packet = 0
            self.error = ''
            self.running = True
        with self.dsp_lock:
            self.dsp.power = self.dsp.frame = None
        try:
            self.command(f'NETSTART {rate} {session}')
            until = time.monotonic() + 3
            while not self.last_packet and time.monotonic() < until:
                time.sleep(.02)
            if not self.last_packet:
                self.command('NET?', 'NET ')
                raise RuntimeError(f'no IQ packets received; device: {self.device}')
        except Exception:
            try:
                self.command('NETSTOP', timeout=1)
            except Exception:
                pass
            self.running = False
            raise
        return self.snapshot()

    def stop(self):
        if self.running:
            self.command('NETSTOP')
            # Drain final in-flight UDP packets before comparing device totals.
            time.sleep(.15)
            self.running = False
            self.command('NET?', 'NET ')
        return self.snapshot()

    def _receive(self):
        while not self.closed.is_set():
            try:
                data, peer = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if peer != self.peer:
                continue
            if not data.startswith(b'S31Q'):
                text = data.decode('ascii', 'replace').strip()
                if text.startswith('NET '):
                    try:
                        self.device = json.loads(text[4:])
                        self.last_device = time.monotonic()
                    except ValueError:
                        continue
                if self.waiting_prefix and text.startswith(('ERR', self.waiting_prefix)):
                    self.replies.put(text)
                elif not text.startswith(('NET ', 'PONG ', 'OK', 'ERR', 'INFO', 'CAPS')):
                    with self.lock:
                        self.continuity.bad_packets += 1
                continue
            try:
                p = decode(data)
            except ValueError:
                with self.lock:
                    self.continuity.bad_packets += 1
                continue
            with self.lock:
                if not self.continuity.accept(p):
                    continue
                self.last_packet = time.monotonic()
                self.chunks.append(p)
                if self.recorder:
                    self.recorder.append(data)

    def _monitor(self):
        while not self.closed.wait(1):
            if self.running:
                try:
                    self.sock.sendto(f'NETPING {self.continuity.session}\n'.encode(), self.peer)
                    # Device telemetry shares this socket, but not command reply queues.
                    with self.command_lock:
                        self.sock.sendto(b'NET?\n', self.peer)
                    if self.last_packet and time.monotonic()-self.last_packet > 2:
                        self.error = 'IQ stream stalled'
                except OSError as e:
                    self.error = str(e)

    def _process(self):
        previous_end = None
        while not self.closed.wait(1/15):
            with self.dsp_lock:
                required = self.dsp.fft_size
                with self.lock:
                    chunks = list(self.chunks)
                if not chunks or chunks[-1].first + chunks[-1].samples == previous_end:
                    continue
                picked, count, next_first = [], 0, None
                for p in reversed(chunks):
                    if next_first is not None and p.first + p.samples != next_first:
                        break
                    picked.append(p.payload)
                    next_first = p.first
                    count += p.samples
                    if count >= required:
                        break
                if count < required:
                    continue
                raw = b''.join(reversed(picked))[-required*2:]
                end = chunks[-1].first + chunks[-1].samples
                discontinuity = previous_end is not None and self.continuity.missing_samples != getattr(self, '_dsp_missing', 0)
                self.dsp.process(raw, chunks[-1].rate, chunks[-1].frequency, end-required, discontinuity)
                self._dsp_missing = self.continuity.missing_samples
                previous_end = end
                self.frame_number += 1

    def configure_dsp(self, fft_size=4096, average=8, remove_dc=True):
        with self.dsp_lock:
            self.dsp.configure(int(fft_size), int(average), remove_dc)

    def record(self, enabled):
        if enabled:
            with self.lock:
                if not self.recorder:
                    self.recording_dir.mkdir(parents=True, exist_ok=True)
                    path = self.recording_dir / (time.strftime('%Y%m%d-%H%M%S') + f'-{secrets.token_hex(2)}.s31iq')
                    self.recorder = Recorder(path)
                return self.recorder.snapshot()
        with self.lock:
            recorder, self.recorder = self.recorder, None
        if recorder:
            self.last_recording = recorder.stop()
        return self.last_recording

    def snapshot(self, include_spectrum=False):
        now = time.monotonic()
        with self.lock:
            result = self.continuity.snapshot()
            record = self.recorder.snapshot() if self.recorder else None
        elapsed = (self.last_packet or now)-self.started if self.started else 0
        device = dict(self.device)
        tail_missing = max(0, device.get('packets', 0)-result['packets']) if not self.running and device.get('session') == result['session'] else 0
        seamless = not any(result[k] for k in ('missing_samples', 'missing_packets', 'bad_packets', 'late_packets', 'dma_drops')) and not device.get('parlio_overflow', 0) and not device.get('dma_drops', 0) and not device.get('tx_errors', 0) and not tail_missing
        result.update(protocol_version=2, payload_checksum='none',
                      running=self.running, elapsed=round(elapsed, 3),
                      mbps=round(result['samples']*16/max(elapsed, .001)/1e6, 3),
                      packet_age=round(now-self.last_packet, 3) if self.last_packet else None,
                      device_age=round(now-self.last_device, 3) if self.last_device else None,
                      transport_continuous=seamless and result['packets'] > 0,
                      final_missing_packets=tail_missing, device=device,
                      socket_buffer=self.receive_buffer, error=self.error, recording=record,
                      last_recording=self.last_recording, frame_number=self.frame_number,
                      settings=self.settings)
        if include_spectrum:
            with self.dsp_lock:
                result['spectrum'] = dict(self.dsp.frame) if self.dsp.frame else None
        return result

    def close(self):
        try:
            self.stop()
        finally:
            self.record(False)
            self.closed.set()
            self.sock.close()
            for t in self.threads:
                t.join(timeout=2)
