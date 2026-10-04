"""Complex, two-sided Hann FFT. Levels are dBFS/bin, not calibrated dBm."""
import numpy as np


def spectrum(iq, fft_size=4096, remove_dc=True):
    x = np.frombuffer(iq, dtype=np.int8).astype(np.float32).reshape(-1, 2)
    if len(x) < fft_size:
        raise ValueError('not enough IQ samples')
    x = x[-fft_size:]
    means = x.mean(axis=0)
    clipping = float(np.mean(np.any((x == -128) | (x == 127), axis=1)))
    z = (x[:, 0] + 1j * x[:, 1]) / 128
    if remove_dc:
        z = z - z.mean()
    window = np.hanning(fft_size)
    power = np.abs(np.fft.fftshift(np.fft.fft(z * window))) ** 2 / window.sum() ** 2
    return power, {'i_dc': float(means[0]), 'q_dc': float(means[1]),
                   'rms_dbfs': float(10 * np.log10(max(float(np.mean(np.abs(z) ** 2)), 1e-16))),
                   'clipping': clipping}


class Processor:
    def __init__(self, fft_size=4096, average=8, remove_dc=True):
        self.configure(fft_size, average, remove_dc)
        self.frame = None

    def configure(self, fft_size, average, remove_dc):
        if fft_size not in (1024, 2048, 4096, 8192, 16384):
            raise ValueError('FFT size must be 1024, 2048, 4096, 8192 or 16384')
        if not 1 <= average <= 100:
            raise ValueError('average must be 1..100')
        self.fft_size, self.average, self.remove_dc = fft_size, average, bool(remove_dc)
        self.power = None

    def process(self, iq, rate, frequency, first, discontinuity=False):
        power, metrics = spectrum(iq, self.fft_size, self.remove_dc)
        if self.power is None or discontinuity:
            self.power = power
        else:
            self.power += (power - self.power) / self.average
        db = 10 * np.log10(np.maximum(self.power, 1e-16))
        # Max pooling preserves narrow peaks in the browser's 1024 display bins.
        bins = db.reshape(1024, -1).max(axis=1)
        self.frame = dict(metrics, db=np.round(bins, 2).tolist(), rate=rate,
                          frequency=frequency, fft_size=self.fft_size, first_sample=first,
                          peak_dbfs=float(db.max()),
                          peak_offset_hz=float((int(db.argmax()) - self.fft_size // 2) * rate / self.fft_size))
        return self.frame
