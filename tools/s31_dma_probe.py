#!/usr/bin/env python3
"""Record bounded S31_DMA_PROBE experiments (requires pyserial).

Examples:
  python tools/s31_dma_probe.py --port COM17 --output results PARLIO 250000 10000 1
  python tools/s31_dma_probe.py --port COM17 --output dma-read DMA 0 1

Build with the S31 pinned SDK, stock defaults, and -DS31_DMA_PROBE=ON.
The diagnostic image must be flashed explicitly; release export rejects it.
DMA mode 3 is an access experiment: completion does not establish correct data.
PARLIO local mode checksums only each node's first 64 bytes; serial mode checks
every payload byte. Neither mode establishes RF waveform continuity.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import zlib


def run(args):
    if args.serial_packages:
        sys.path.insert(0, str(args.serial_packages.resolve()))
    import serial

    result = dict(command=' '.join(args.command), port=args.port, lines=[],
                  frames=0, bytes=0, crc_failures=0, sequence_gaps=0)
    args.output.mkdir(parents=True, exist_ok=False)
    connection = serial.Serial()
    connection.port, connection.baudrate = args.port, args.baud
    connection.timeout, connection.write_timeout = .2, 2
    connection.dtr = connection.rts = False

    def line():
        data = bytearray()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and len(data) < 1024:
            byte = connection.read(1)
            data.extend(byte)
            if byte == b'\n':
                return data.decode('ascii').strip()
        raise TimeoutError(f'incomplete response: {data!r}')

    def payload(size):
        if not 0 < size <= 4032:
            raise ValueError(f'unexpected payload size: {size}')
        data = bytearray()
        deadline = time.monotonic() + 5
        while len(data) < size and time.monotonic() < deadline:
            data.extend(connection.read(size - len(data)))
        if len(data) != size:
            raise TimeoutError(f'incomplete payload: {len(data)}/{size}')
        return data

    previous, aggregate = -1, 0
    try:
        connection.open()
        connection.reset_input_buffer()
        connection.write((result['command'] + '\n').encode('ascii'))
        connection.flush()
        start = time.perf_counter()
        with (args.output / 'samples.bin').open('wb') as samples:
            while True:
                response = line()
                fields = response.split()
                if fields and fields[0] == 'PDATA':
                    if len(fields) != 4:
                        raise ValueError(response)
                    sequence, size, expected = int(fields[1]), int(fields[2]), int(fields[3], 16)
                    data = payload(size)
                    samples.write(data)
                    result['crc_failures'] += zlib.crc32(data) != expected
                    if sequence <= previous:
                        raise ValueError(f'out-of-order sequence: {sequence} after {previous}')
                    result['sequence_gaps'] += sequence - previous - 1
                    previous = sequence
                    aggregate = zlib.crc32(data, aggregate)
                    result['frames'] += 1
                    result['bytes'] += size
                    continue
                result['lines'].append(response)
                if fields and fields[0] == 'PEND':
                    if len(fields) != 9:
                        raise ValueError(response)
                    result['device'] = dict(zip(
                        ['rate_hz', 'requested_ms', 'events', 'copied_bytes', 'dropped_nodes',
                         'max_backlog', 'elapsed_us'], map(int, fields[1:8])))
                    result['device']['crc32'] = fields[8]
                elif response.startswith('ERR'):
                    raise RuntimeError(response)
                elif not response.startswith('DMA '):
                    raise ValueError(f'unexpected response: {response}')
                break
            if args.command[0] == 'PARLIO':
                if args.command[-1] == '0':
                    response = line()
                    result['lines'].append(response)
                    fields = response.split()
                    if len(fields) != 3 or fields[0] != 'PSAMPLE':
                        raise ValueError(response)
                    data = payload(int(fields[1]))
                    samples.write(data)
                    if zlib.crc32(data) != int(fields[2], 16):
                        raise ValueError('stopped-ring sample CRC mismatch')
                    result['crc_scope'] = 'local: first 64 bytes per consumed DMA node'
                else:
                    result['crc_scope'] = 'all uploaded bytes'
                    result['aggregate_crc32'] = f'{aggregate:08x}'
                    if (result['bytes'] != result['device']['copied_bytes'] or
                            result['aggregate_crc32'] != result['device']['crc32']):
                        raise ValueError('device/host byte count or aggregate CRC mismatch')
        result['elapsed_s'] = time.perf_counter() - start
        connection.write(b'RELEASE\n')
        connection.flush()
        result['release'] = line()
        if result['release'] != 'OK':
            raise ValueError(result['release'])
    except Exception as error:
        result['error'] = repr(error)
        raise
    finally:
        connection.close()
        (args.output / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--baud', type=int, default=2000000)
    parser.add_argument('--output', type=Path, required=True, help='New results directory')
    parser.add_argument('--serial-packages', type=Path, help='Optional pyserial installation directory')
    parser.add_argument('command', nargs='+')
    options = parser.parse_args()
    if options.command[0] not in ('DMA', 'PARLIO'):
        parser.error('Choose DMA or PARLIO')
    run(options)
