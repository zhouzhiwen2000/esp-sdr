"""Validate a .s31iq recording and export contiguous signed I8/Q8 samples."""
import argparse
import json
import os
from pathlib import Path
import struct
import tempfile

from .protocol import Continuity, Packet, RATES, decode


def decode_legacy_record(data):
    """Read old files only; live receivers use the checksum-free v2 protocol."""
    import zlib
    header = struct.Struct('<4sBBHIIQIIHHQI')
    if len(data) < header.size:
        raise ValueError('short legacy header')
    magic, version, flags, size, session, seq, first, rate, freq, count, fmt, drops, crc = header.unpack_from(data)
    if (magic, version, flags, size, fmt) != (b'S31Q', 1, 1, 48, 1):
        raise ValueError('unsupported legacy header')
    if not count or len(data) != size + count * 2 or rate not in RATES:
        raise ValueError('invalid legacy payload')
    payload = memoryview(data)[size:]
    if zlib.crc32(payload, zlib.crc32(memoryview(data)[:44])) != crc:
        raise ValueError('legacy CRC mismatch')
    return Packet(session, seq, first, rate, freq, count, drops, payload)


def read_packets(path):
    with Path(path).open('rb') as stream:
        signature = stream.read(8)
        if signature not in (b'S31IQ\x01\x00\x00', b'S31IQ\x02\x00\x00'):
            raise ValueError('not an S31IQ recording')
        legacy = signature[5] == 1
        header_size = 48 if legacy else 44
        while True:
            size = stream.read(2)
            if not size:
                return
            if len(size) != 2:
                raise ValueError('truncated record length')
            length, = struct.unpack('<H', size)
            if not header_size < length <= header_size + 1344:
                raise ValueError('invalid record length')
            data = stream.read(length)
            if len(data) != length:
                raise ValueError('truncated record')
            yield decode_legacy_record(data) if legacy else decode(data)


def export_ci8(source, target):
    with Path(source).open('rb') as stream:
        legacy = stream.read(8) == b'S31IQ\x01\x00\x00'
    target = Path(target)
    metadata_path = target.with_suffix(target.suffix+'.json')
    if target.exists() or metadata_path.exists():
        raise FileExistsError('output or metadata already exists')
    count, first, tracker = 0, None, None
    name = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as output:
            name = Path(output.name)
            for p in read_packets(source):
                if first is None:
                    first = p
                    tracker = Continuity(p.session)
                    tracker.next_sequence, tracker.next_sample = p.sequence, p.first
                if p.rate != first.rate or p.frequency != first.frequency or not tracker.accept(p):
                    raise ValueError('recording contains multiple sessions or changed parameters')
                if tracker.missing_samples or tracker.missing_packets:
                    raise ValueError('recording contains gaps; keep .s31iq metadata instead of flattening')
                output.write(p.payload)
                count += p.samples
        if first is None:
            raise ValueError('empty recording')
        metadata = dict(format='ci8', sample_rate=first.rate, center_frequency_hz=first.frequency*1_000_000,
                        first_sample=first.first, samples=count, session=first.session,
                        source=str(source), crc_verified=legacy,
                        payload_checksum='crc32' if legacy else 'none', continuous=True)
        # Exclusive creation prevents replacing a concurrently created output.
        os.link(name, target)
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        return metadata
    finally:
        if name is not None:
            name.unlink(missing_ok=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('output', type=Path, help='new .ci8 output file')
    a = p.parse_args()
    print(json.dumps(export_ci8(a.source, a.output), indent=2))


if __name__ == '__main__':
    main()
