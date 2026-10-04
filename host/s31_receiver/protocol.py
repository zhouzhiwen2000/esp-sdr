"""S31Q v2: little-endian metadata and interleaved signed I8/Q8."""
from dataclasses import dataclass
import struct

HEADER = struct.Struct('<4sBBHIIQIIHHQ')
RATES = (250_000, 1_000_000, 2_000_000, 4_000_000, 8_000_000, 16_000_000, 20_000_000, 32_000_000, 40_000_000, 53_333_333)


@dataclass(slots=True)
class Packet:
    session: int
    sequence: int
    first: int
    rate: int
    frequency: int
    samples: int
    dma_drops: int
    payload: memoryview


def decode(data: bytes) -> Packet:
    if len(data) < HEADER.size:
        raise ValueError('short header')
    magic, version, flags, size, session, seq, first, rate, freq, count, fmt, drops = HEADER.unpack_from(data)
    if magic != b'S31Q' or version != 2 or flags != 0 or size != HEADER.size or fmt != 1:
        raise ValueError('unsupported header')
    if not count or len(data) != size + count * 2 or rate not in RATES:
        raise ValueError('invalid payload length or rate')
    payload = memoryview(data)[size:]
    return Packet(session, seq, first, rate, freq, count, drops, payload)


class Continuity:
    """Count missing samples without inserting silence or hiding reordering."""
    def __init__(self, session):
        self.session = session
        self.next_sequence = self.next_sample = 0
        self.packets = self.samples = self.missing_packets = self.missing_samples = 0
        self.late_packets = self.foreign_packets = self.bad_packets = self.dma_drops = 0

    def accept(self, p: Packet) -> bool:
        if p.session != self.session:
            self.foreign_packets += 1
            return False
        delta = (p.sequence - self.next_sequence) & 0xffffffff
        if delta >= 0x80000000 or p.first < self.next_sample:
            self.late_packets += 1
            return False
        self.missing_packets += delta
        self.missing_samples += p.first - self.next_sample
        self.next_sequence = (p.sequence + 1) & 0xffffffff
        self.next_sample = p.first + p.samples
        self.dma_drops = max(self.dma_drops, p.dma_drops)
        self.packets += 1
        self.samples += p.samples
        return True

    def snapshot(self):
        return {key: getattr(self, key) for key in (
            'session', 'packets', 'samples', 'missing_packets', 'missing_samples',
            'late_packets', 'foreign_packets', 'bad_packets', 'dma_drops', 'next_sample')}
