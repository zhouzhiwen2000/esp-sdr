from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'host'))
from s31_receiver.protocol import HEADER, Continuity, decode
from s31_receiver.dsp import Processor, spectrum
from s31_receiver.recording import export_ci8
import numpy as np


def packet(seq=0, first=0, session=123, count=672, drops=0):
    payload = bytes((i % 256 for i in range(count*2)))
    h = HEADER.pack(b'S31Q', 2, 0, 44, session, seq, first, 4_000_000, 2412, count, 1, drops)
    return h + payload


class ProtocolTests(unittest.TestCase):
    def test_header_and_truncation(self):
        data = packet()
        p = decode(data)
        self.assertEqual((p.sequence, p.samples, len(p.payload)), (0, 672, 1344))
        for index in (0, 4, 5, 6, 24, 32, 34):
            bad = bytearray(data); bad[index] ^= 1
            with self.assertRaises(ValueError):
                decode(bad)
        for size in (0, 43, 44, len(data)-1):
            with self.assertRaises(ValueError):
                decode(data[:size])

    def test_loss_late_foreign_and_dma(self):
        c = Continuity(123)
        self.assertTrue(c.accept(decode(packet())))
        self.assertTrue(c.accept(decode(packet(2, 2016, drops=672))))
        self.assertEqual((c.missing_packets, c.missing_samples, c.dma_drops), (1, 1344, 672))
        self.assertFalse(c.accept(decode(packet(1, 672))))
        self.assertFalse(c.accept(decode(packet(3, 2688, session=456))))
        self.assertEqual((c.late_packets, c.foreign_packets, c.samples), (1, 1, 1344))

    def test_initial_loss_and_sequence_wrap(self):
        c = Continuity(123)
        self.assertTrue(c.accept(decode(packet(3, 2016))))
        self.assertEqual((c.missing_packets, c.missing_samples), (3, 2016))
        c.next_sequence = 0xffffffff
        self.assertTrue(c.accept(decode(packet(0xffffffff, 2688))))
        self.assertTrue(c.accept(decode(packet(0, 3360))))
        self.assertEqual(c.missing_packets, 3)

    def test_fft_frequency_amplitude_dc_and_clipping(self):
        n = 4096
        tone = .5 * np.exp(2j*np.pi*137*np.arange(n)/n)
        iq = np.column_stack((np.round(tone.real*128)+20, np.round(tone.imag*128)-10)).astype('int8').tobytes()
        power, metrics = spectrum(iq, n, True)
        self.assertEqual(int(power.argmax()), n//2+137)
        self.assertAlmostEqual(10*np.log10(power.max()), -6.0206, delta=.05)
        self.assertAlmostEqual(metrics['i_dc'], 20, delta=.02)
        self.assertEqual(metrics['clipping'], 0)
        self.assertLess(power[n//2], 1e-10)
        p = Processor(n, 8)
        frame = p.process(iq, 4_000_000, 2412, 0)
        self.assertEqual(len(frame['db']), 1024)
        self.assertAlmostEqual(frame['peak_offset_hz'], 137*4_000_000/n)

    def test_recording_export_and_gap_rejection(self):
        with tempfile.TemporaryDirectory() as temp:
            source, target = Path(temp)/'source.s31iq', Path(temp)/'out.ci8'
            def write(packets):
                source.write_bytes(b'S31IQ\x02\x00\x00' + b''.join(struct.pack('<H',len(p))+p for p in packets))
            # A recording can begin midway through a live session.
            write([packet(10, 6720), packet(11, 7392)])
            info = export_ci8(source, target)
            self.assertEqual((info['samples'], info['first_sample'], target.stat().st_size), (1344, 6720, 2688))
            self.assertFalse(info['crc_verified'])
            self.assertEqual(info['payload_checksum'], 'none')
            with self.assertRaises(FileExistsError):
                export_ci8(source, target)
            target.unlink(); target.with_suffix('.ci8.json').unlink()
            for data in ([packet(10, 6720), packet(12, 8064)],
                         [packet(), packet(1, 672, session=456)],
                         [packet(), packet(1, 672)[:-1]]):
                write(data)
                with self.assertRaises(ValueError):
                    export_ci8(source, target)
                self.assertFalse(target.exists())

    def test_payload_has_no_application_checksum(self):
        data = bytearray(packet())
        data[-1] ^= 1
        self.assertEqual(decode(data).payload[-1], data[-1])
        old = bytearray(packet()); old[4] = 1
        with self.assertRaises(ValueError):
            decode(old)

    def test_legacy_recording_export(self):
        with tempfile.TemporaryDirectory() as temp:
            source, target = Path(temp)/'old.s31iq', Path(temp)/'old.ci8'
            payload = bytes(range(32))
            h = struct.pack('<4sBBHIIQIIHHQI', b'S31Q', 1, 1, 48, 9, 0, 0,
                            4_000_000, 2412, 16, 1, 0, 0)
            frame = h[:44] + struct.pack('<I', zlib.crc32(payload, zlib.crc32(h[:44]))) + payload
            source.write_bytes(b'S31IQ\x01\x00\x00' + struct.pack('<H', len(frame)) + frame)
            info = export_ci8(source, target)
            self.assertTrue(info['crc_verified'])
            self.assertEqual(target.read_bytes(), payload)
            corrupt = bytearray(source.read_bytes()); corrupt[-1] ^= 1; source.write_bytes(corrupt)
            with self.assertRaises(ValueError):
                export_ci8(source, Path(temp)/'bad.ci8')


if __name__ == '__main__':
    unittest.main()
