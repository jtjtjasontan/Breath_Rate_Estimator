"""
breath_analyzer.py
==================
Real-time dual-sensor breath analyser for the STM32 +
thermistor (TMP61, CH5) + conductive rubber strain band (CH6) rig.

This file supersedes both `breath_live_plot.py` (live oscilloscope only)
and `breath_recorder_pro.py` (start/stop protocol, incompatible with the
new continuous-stream firmware) for everything except the docstrings and
plot styling, which are inherited.

Pipeline per channel
--------------------
    raw → DC removal → moving-average (FIR, 10-tap) → Sym4 wavelet
        ↳ thermistor : scipy.signal.find_peaks → cycle count + BPM
        ↳ rubber     : positive-going zero-crossings on cleaned signal
                       (with min-distance refractory)
                       → cycle count + BPM
                       Hanning FFT used only for confidence (spectral SNR)
                       and live spectrum display.
    BPM_thermistor, BPM_rubber  →  confidence-weighted decision fusion
                                →  BPM_fused, count_fused

The rubber estimator was simplified once the differential-amplifier stage
in the analog front-end was added: the differential output is already a
near-square wave with sharp transitions at each breath cycle, so the
earlier Butterworth-bandpass / Hilbert-envelope / amplitude-gate cascade
became unnecessary.

Modes
-----
    --mode rest      Resting   : ~6–30 BPM band, peak min-distance 2.0 s
    --mode exercise  Exercise  : ~18–60 BPM band, peak min-distance 0.8 s

The mode sets the FFT search range and the peak-detection min-distance.

In-lab demo flow
----------------
    1.  Strap on sensors, start the script:
            python breath_analyzer.py --port COM3 --mode rest
    2.  Live BPM appears at the top of the window for each sensor + the
        fused estimate.  Wait for them to stabilise.
    3.  Press R to start a recording.  Status turns red.
            • Rest mode      — press R again after the 5th metronome cycle
            • Exercise mode  — recording auto-stops after 30 s
    4.  After recording stops:
            • A timestamped CSV is saved to ./data/captures/
            • The terminal prints the cycle counts and BPMs
            • Edit the CSV header line `# ground_truth_count: ?` with
              your reference value before running analyze_session.py
    5.  Press R again for the next recording, or Q to quit.

Dependencies
------------
    pip install pyserial matplotlib numpy scipy PyWavelets
"""

import argparse
import csv
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import pywt
from scipy.signal import find_peaks

import matplotlib
# NB: backend selection happens in main(), not at import time, so this module
# can be imported by analyze_session.py (or any headless script) without
# forcing TkAgg, which would crash on systems without tkinter.
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
SAMPLE_RATE_HZ = 50          # must match STM32 TIM6 rate
ADC_MAX        = 4095        # 12-bit ADC
VREF           = 3.3         # STM32 reference voltage

EXERCISE_RECORD_SEC = 30     # demo: count manual breaths over 30 s after exertion

# Live BPM is computed on this much of the rolling buffer
ANALYSIS_WINDOW_SEC = 15     # ≥3 full cycles at 12 BPM

# Colours
COLOUR_RAW   = "#bdbdbd"     # raw ADC (background trace)
COLOUR_CH5   = "#4c9be8"     # cleaned thermistor (blue)
COLOUR_CH6   = "#e87c4c"     # cleaned rubber (orange)
COLOUR_PEAK  = "#2ca02c"     # detected peaks (green)
COLOUR_ZC    = "#9467bd"     # detected zero-crossings (purple)
COLOUR_FFT   = "#444444"
COLOUR_FPEAK = "#d62728"     # dominant FFT bin (red)
COLOUR_REC   = "#d62728"     # recording status text

OUTPUT_DIR = Path("data/captures")

# Per-mode tuning.  Bands and min-distances are sized so the FFT band exactly
# straddles the realistic breathing rate for that condition and the peak / ZC
# filter can't double-trigger inside one cycle.
MODE_PARAMS = {
    "rest": {
        "label":          "Resting",
        "breath_hz_min":  0.08,   #  4.8 BPM
        "breath_hz_max":  0.60,   # 36 BPM
        "peak_min_dist":  2.0,    # seconds — caps detection at 30 BPM
        "expected_range": "4.8 – 36 BPM",
    },
    "exercise": {
        "label":          "Exercise",
        "breath_hz_min":  0.30,   # 18 BPM
        "breath_hz_max":  1.50,   # 90 BPM
        "peak_min_dist":  0.8,    # seconds — caps detection at 75 BPM
        "expected_range": "18 – 90 BPM",
    },
}

# ──────────────────────────────────────────────────────────────────────────────
# Shared state — thread-safe rolling buffer with recording support
# ──────────────────────────────────────────────────────────────────────────────
class SensorBuffer:
    """
    Thread-safe rolling buffer for both ADC channels.

    `ch5`/`ch6` are bounded deques — appending after `maxlen` automatically
    evicts the oldest sample, giving a free rolling window with no slicing.

    A second pair of unbounded lists (`rec_ch5`, `rec_ch6`) captures samples
    while `recording` is True so a full demo trial can be post-processed at
    full duration regardless of the rolling-display window length.
    """

    def __init__(self, max_samples: int):
        self.ch5 = deque([0.0] * max_samples, maxlen=max_samples)
        self.ch6 = deque([0.0] * max_samples, maxlen=max_samples)
        self.lock = threading.Lock()

        self.total_samples = 0
        self.dropped_lines = 0
        self.connected     = False

        # Recording state
        self.recording  = False
        self.rec_ch5    = []
        self.rec_ch6    = []
        self.rec_start  = 0.0

    def push(self, ch5_val: int, ch6_val: int) -> None:
        with self.lock:
            self.ch5.append(ch5_val)
            self.ch6.append(ch6_val)
            self.total_samples += 1
            if self.recording:
                self.rec_ch5.append(ch5_val)
                self.rec_ch6.append(ch6_val)

    def snapshot(self):
        """Lock-safe copy of both rolling deques."""
        with self.lock:
            return (np.array(self.ch5, dtype=float),
                    np.array(self.ch6, dtype=float))

    def start_recording(self):
        with self.lock:
            self.rec_ch5 = []
            self.rec_ch6 = []
            self.recording = True
            self.rec_start = time.time()

    def stop_recording(self):
        with self.lock:
            self.recording = False
            return (np.array(self.rec_ch5, dtype=float),
                    np.array(self.rec_ch6, dtype=float),
                    self.rec_start)


# ──────────────────────────────────────────────────────────────────────────────
# Serial reader thread
# ──────────────────────────────────────────────────────────────────────────────
def serial_reader(ser: serial.Serial,
                  buf: SensorBuffer,
                  stop_event: threading.Event) -> None:
    """Daemon thread — reads `CH5,CH6\\r\\n` lines and pushes them into the buffer."""
    buf.connected = True
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            buf.connected = False
            stop_event.set()
            break

        if not raw:
            continue

        line = raw.decode("ascii", errors="ignore").strip()
        if "," not in line:
            continue

        parts = line.split(",")
        if len(parts) != 2:
            buf.dropped_lines += 1
            continue

        try:
            ch5_val = int(parts[0])
            ch6_val = int(parts[1])
        except ValueError:
            buf.dropped_lines += 1
            continue

        ch5_val = max(0, min(ADC_MAX, ch5_val))
        ch6_val = max(0, min(ADC_MAX, ch6_val))
        buf.push(ch5_val, ch6_val)

    buf.connected = False


# ──────────────────────────────────────────────────────────────────────────────
# Common DSP preprocessing pipeline (shared by both channels)
# ──────────────────────────────────────────────────────────────────────────────
def preprocess_signal(adc_values, apply_window: bool = False) -> np.ndarray:
    """
    Reference DSP pipeline:

        1. DC offset removal         — centres at zero, kills 0 Hz spike
        2. 10-tap moving average     — FIR low-pass for quantisation noise
        3. Sym4 wavelet denoising    — soft-thresholding of detail coeffs
        4. (optional) Hanning window — tapers edges for FFT only
    """
    arr = np.asarray(adc_values, dtype=float)
    n = len(arr)
    if n < 32:                              # too short for level-2 wavelet
        return arr - arr.mean() if n else arr

    # [1] DC removal
    arr = arr - np.mean(arr)

    # [2] Moving average
    kernel = np.ones(10) / 10.0
    arr = np.convolve(arr, kernel, mode="same")

    # [3] Sym4 wavelet denoising (universal-threshold soft-thresholding)
    coeffs = pywt.wavedec(arr, "sym4", level=2)
    sigma   = np.median(np.abs(coeffs[-1])) / 0.6745
    uthresh = sigma * np.sqrt(2.0 * np.log(n))
    coeffs[1:] = [pywt.threshold(c, value=uthresh, mode="soft") for c in coeffs[1:]]
    arr = pywt.waverec(coeffs, "sym4")[:n]   # trim any wavelet length drift

    # [4] Hanning window (only on the FFT path)
    if apply_window:
        arr = arr * np.hanning(n)

    return arr


# ──────────────────────────────────────────────────────────────────────────────
# Thermistor — peak counting
# ──────────────────────────────────────────────────────────────────────────────
def estimate_thermistor(raw: np.ndarray, fs: int, mode: str):
    """
    Thermistor cycles are detected by peak counting on the cleaned signal.
    The waveform is clean and roughly sinusoidal: one breath ↔ one peak.

    Returns
    -------
    count : int            number of breath cycles inside the analysed window
    bpm   : float          count × 60 / duration_s
    peak_idx : np.ndarray  sample indices of the detected peaks
    cleaned : np.ndarray   DC-restored cleaned signal (same scale as raw)
    conf  : float          mean peak prominence / noise-floor estimate
    fft_freqs, fft_mag : np.ndarray  spectrum of the cleaned signal
    """
    blank_freqs = np.array([0.0])
    blank_mag   = np.array([0.0])
    if len(raw) < fs * 5:                  # need ≥5 s
        return (0, 0.0, np.array([], int),
                np.asarray(raw, dtype=float).copy(),
                0.0, blank_freqs, blank_mag)

    raw_arr = np.asarray(raw, dtype=float)
    raw_mean = float(raw_arr.mean())
    cleaned_centred = preprocess_signal(raw_arr, apply_window=False)
    # Cleaned signal for plotting — restore the DC offset so it overlays raw.
    cleaned = cleaned_centred + raw_mean

    # FFT on the cleaned (DC-removed) signal for the CH5 spectrum panel.
    n = len(cleaned_centred)
    windowed_t = cleaned_centred * np.hanning(n)
    fft_mag_t  = np.abs(np.fft.rfft(windowed_t))
    freqs_t    = np.fft.rfftfreq(n, d=1.0 / fs)

    mp = MODE_PARAMS[mode]
    min_dist = int(mp["peak_min_dist"] * fs)

    # Adaptive prominence: combine a noise-only proxy with a fraction of
    # the signal amplitude.  The MAD of *first differences* is the right
    # noise proxy here — consecutive samples on a slow breath signal are
    # nearly equal, so diff-MAD reflects only the residual high-freq noise,
    # while the MAD of the signal itself would be dominated by the breath
    # swing (≈ 0.7 × amplitude for a sinusoid).
    diff_mad = np.median(np.abs(np.diff(cleaned_centred))) + 1e-6
    sig_std  = float(np.std(cleaned_centred))
    prominence = max(8.0 * diff_mad, 0.20 * sig_std, 10.0)

    peaks, props = find_peaks(cleaned_centred, distance=min_dist, prominence=prominence)

    duration_s = len(raw) / fs
    bpm   = (len(peaks) / duration_s) * 60.0 if duration_s > 0 else 0.0
    # Confidence = mean peak prominence relative to the noise floor estimate.
    conf  = float(props["prominences"].mean() / (8.0 * diff_mad)) if len(peaks) else 0.0

    return int(len(peaks)), bpm, peaks, cleaned, conf, freqs_t, fft_mag_t


# ──────────────────────────────────────────────────────────────────────────────
# Conductive rubber — direct positive-going zero-crossing count
# ──────────────────────────────────────────────────────────────────────────────
def estimate_rubber(raw: np.ndarray, fs: int, mode: str):
    """
    Breath-cycle counter for the conductive-rubber strain signal.

    The differential-amplifier hardware stage already produces a near-square
    wave with sharp rail-to-rail transitions at each breath cycle, so breath
    events can be counted directly from positive-going zero-crossings of the
    cleaned signal — no bandpass / envelope / amplitude gating required.

    Method
    ------
    1. Shared DSP pipeline: DC removal → 10-tap moving average → Sym4
       wavelet denoising (via preprocess_signal()).
    2. Hanning-windowed FFT identifies the dominant in-band frequency
       f_peak.  This drives two things: the rubber-branch confidence
       (peak-to-mean in-band power ratio) and the zero-crossing refractory
       in step 3.
    3. Direct positive-going zero-crossing count on the cleaned signal,
       with a refractory window of 0.7 × (1/f_peak) seconds — wide enough
       to suppress intra-cycle wiggles that survive the analog stage,
       narrow enough to allow rate variation between cycles.
    4. BPM is derived from count / duration_s — no longer tied to the
       FFT's bin resolution.

    Returns
    -------
    count : int            positive-going zero-crossing count (= breath cycles)
    bpm   : float          count × 60 / duration_s
    zc_idx : np.ndarray    sample indices of accepted zero-crossings
    cleaned : np.ndarray   DSP-cleaned signal, DC-restored for display
    f_peak : float         dominant in-band frequency [Hz]   (display only)
    conf  : float          peak-FFT power / mean in-band power (spectral SNR)
    fft_freqs, fft_mag : np.ndarray  spectrum (for plotting)
    """
    blank = (0, 0.0, np.array([], int),
             np.asarray(raw, dtype=float).copy(),
             0.0, 0.0,
             np.array([0.0]), np.array([0.0]))
    if len(raw) < fs * 5:                  # need ≥5 s
        return blank

    raw_arr  = np.asarray(raw, dtype=float)
    raw_mean = float(raw_arr.mean())
    cleaned_centred = preprocess_signal(raw_arr, apply_window=False)
    cleaned = cleaned_centred + raw_mean    # DC-restored for display

    # ── FFT (Hanning-windowed) — confidence metric + spectrum display ──────
    n = len(cleaned_centred)
    windowed = cleaned_centred * np.hanning(n)
    fft_mag  = np.abs(np.fft.rfft(windowed))
    freqs    = np.fft.rfftfreq(n, d=1.0 / fs)

    mp   = MODE_PARAMS[mode]
    band = (freqs >= mp["breath_hz_min"]) & (freqs <= mp["breath_hz_max"])
    if band.any() and fft_mag[band].max() >= 1e-3:
        f_peak     = float(freqs[band][np.argmax(fft_mag[band])])
        peak_power = float(fft_mag[band].max())
        mean_power = float(fft_mag[band].mean())
        conf       = peak_power / (mean_power + 1e-6)        # spectral SNR
    else:
        f_peak, conf = 0.0, 0.0

    # ── Positive-going zero-crossings of the cleaned signal ────────────────
    # sign(0) == 0 in numpy, which would cause a 0→+1 transition to register
    # as both crossings.  diff(sign) > 0 handles this correctly: only
    # transitions from non-positive to positive count.
    sign_change = np.where(np.diff(np.sign(cleaned_centred)) > 0)[0]

    # Refractory window:  the cleaned rubber signal often contains multiple
    # zero-crossings inside one breath cycle (small intra-cycle wiggles the
    # analog stage doesn't fully suppress).  The FFT has already identified
    # the dominant breath period `1/f_peak`, so we set the refractory to
    # 70 % of that period — wide enough to suppress every intra-cycle
    # crossing, narrow enough to allow legitimate rate variation between
    # cycles.  If the FFT failed to find an in-band peak (f_peak == 0),
    # fall back to the mode's static min-distance value.
    if f_peak > 0:
        min_dist = int(0.7 * (1.0 / f_peak) * fs)
    else:
        min_dist = int(mp["peak_min_dist"] * fs)
    if len(sign_change) > 1:
        kept = [int(sign_change[0])]
        for idx in sign_change[1:]:
            if idx - kept[-1] >= min_dist:
                kept.append(int(idx))
        zcs = np.array(kept, dtype=int)
    else:
        zcs = sign_change.astype(int)

    duration_s = len(raw) / fs
    count = int(len(zcs))
    bpm   = (count / duration_s) * 60.0 if duration_s > 0 else 0.0

    return (count, bpm, zcs, cleaned, f_peak, conf, freqs, fft_mag)


# ──────────────────────────────────────────────────────────────────────────────
# Decision fusion
# ──────────────────────────────────────────────────────────────────────────────
def fuse_decisions(bpm_t: float, conf_t: float, count_t: int,
                   bpm_r: float, conf_r: float, count_r: int):
    """
    Confidence-weighted decision-level fusion.

    Hard dropout thresholds are intentionally absent — the rubber sensor's
    weight (n_r) is always counted proportional to its quality, preventing
    it from dropping abruptly to zero on a single bad sample.
    """
    # Scaling factor that keeps the rubber sensor's weighting lower overall
    # than the thermistor's, since empirically the thermistor produces the
    # more reliable BPM estimate.  Never reaches zero, so the rubber
    # always contributes proportionally to its confidence.
    RUBBER_MULTIPLIER = 0.5

    # Use max(..., 1e-5) as a tiny baseline floor to guarantee that
    # the contribution is always mathematically counted, even if confidence
    # hits absolute zero.
    w_t = max(conf_t, 1e-5)
    w_r = max(conf_r, 1e-5) * RUBBER_MULTIPLIER

    # Calculate smooth relative weights (convex combination)
    total = w_t + w_r
    n_t   = w_t / total
    n_r   = w_r / total

    # Fused outputs
    bpm_fused   = n_t * bpm_t + n_r * bpm_r
    count_fused = int(round(n_t * count_t + n_r * count_r))

    # Set a flag to warn the UI if either sensor drops below standard tracking quality
    low_conf = (conf_t < 2.0 or conf_r < 2.0)

    return bpm_fused, count_fused, (n_t, n_r), low_conf


# ──────────────────────────────────────────────────────────────────────────────
# Plot setup
# ──────────────────────────────────────────────────────────────────────────────
def build_figure(window_s: int, mode: str):
    """
    Build the 2×2 live-display figure and return every artist that needs
    to be updated each animation frame.

    Layout
    ------
        ┌──────────────────────┬──────────────────────┐
        │ CH5 waveform + peaks │ CH5 FFT (full)       │
        ├──────────────────────┼──────────────────────┤
        │ CH6 waveform + ZCs   │ CH6 FFT + f_peak     │
        └──────────────────────┴──────────────────────┘
                       suptitle: live BPMs + status
    """
    mp = MODE_PARAMS[mode]
    max_samples = window_s * SAMPLE_RATE_HZ
    t_axis = np.linspace(-window_s, 0.0, max_samples)

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 7.5),
                             gridspec_kw={"width_ratios": [3, 2]})
    fig.subplots_adjust(left=0.06, right=0.96, top=0.86,
                        bottom=0.08, hspace=0.40, wspace=0.28)
    (ax_w5, ax_f5), (ax_w6, ax_f6) = axes

    # ── CH5 waveform — thermistor ──────────────────────────────────────────
    ax_w5.set_title(f"CH5 — Thermistor (TMP61)  ·  raw + cleaned + peaks",
                    fontsize=10, loc="left")
    ax_w5.set_xlim(-window_s, 0)
    ax_w5.set_xlabel("time before now [s]", fontsize=9)
    ax_w5.set_ylabel("ADC counts", fontsize=9, color=COLOUR_CH5)
    ax_w5.grid(True, alpha=0.25)
    l5_raw,   = ax_w5.plot(t_axis, np.zeros(max_samples), color=COLOUR_RAW,
                           linewidth=0.8, alpha=0.9, label="raw")
    l5_clean, = ax_w5.plot(t_axis, np.zeros(max_samples), color=COLOUR_CH5,
                           linewidth=1.6, label="cleaned")
    l5_peaks, = ax_w5.plot([], [], "v", color=COLOUR_PEAK, markersize=8,
                           markeredgecolor="black", markeredgewidth=0.5,
                           label="peaks")
    ax_w5.legend(loc="upper right", fontsize=8, framealpha=0.85)

    # ── CH5 FFT — for reference / comparison only ──────────────────────────
    ax_f5.set_title("CH5 spectrum (Hanning FFT)", fontsize=10, loc="left")
    ax_f5.set_xlim(0, mp["breath_hz_max"] * 1.6)
    ax_f5.set_xlabel("frequency [Hz]", fontsize=9)
    ax_f5.set_ylabel("magnitude", fontsize=9)
    ax_f5.grid(True, alpha=0.25)
    l5_fft, = ax_f5.plot([0], [0], color=COLOUR_FFT, linewidth=1.0)
    ax_f5.axvspan(mp["breath_hz_min"], mp["breath_hz_max"],
                  facecolor=COLOUR_CH5, alpha=0.08)

    # ── CH6 waveform — rubber band ─────────────────────────────────────────
    ax_w6.set_title(f"CH6 — Conductive rubber  ·  raw + cleaned + ZCs",
                    fontsize=10, loc="left")
    ax_w6.set_xlim(-window_s, 0)
    ax_w6.set_xlabel("time before now [s]", fontsize=9)
    ax_w6.set_ylabel("ADC counts (centred)", fontsize=9, color=COLOUR_CH6)
    ax_w6.grid(True, alpha=0.25)
    l6_raw,   = ax_w6.plot(t_axis, np.zeros(max_samples), color=COLOUR_RAW,
                           linewidth=0.8, alpha=0.9, label="raw")
    l6_clean, = ax_w6.plot(t_axis, np.zeros(max_samples), color=COLOUR_CH6,
                           linewidth=1.4, label="cleaned")
    l6_zcs,   = ax_w6.plot([], [], "o", color=COLOUR_ZC, markersize=6,
                           markeredgecolor="black", markeredgewidth=0.5,
                           label="zero-crossings")
    ax_w6.legend(loc="upper right", fontsize=8, framealpha=0.85)

    # ── CH6 FFT — annotated with the dominant frequency ────────────────────
    ax_f6.set_title("CH6 spectrum  ·  red dot = f_peak", fontsize=10, loc="left")
    ax_f6.set_xlim(0, mp["breath_hz_max"] * 1.6)
    ax_f6.set_xlabel("frequency [Hz]", fontsize=9)
    ax_f6.set_ylabel("magnitude", fontsize=9)
    ax_f6.grid(True, alpha=0.25)
    l6_fft,  = ax_f6.plot([0], [0], color=COLOUR_FFT, linewidth=1.0)
    l6_peak, = ax_f6.plot([], [], "o", color=COLOUR_FPEAK, markersize=10,
                         markeredgecolor="black", markeredgewidth=0.7)
    ax_f6.axvspan(mp["breath_hz_min"], mp["breath_hz_max"],
                  facecolor=COLOUR_CH6, alpha=0.08)
    ann_f6 = ax_f6.annotate("", xy=(0, 0), xytext=(8, -8),
                            textcoords="offset points", fontsize=9,
                            color=COLOUR_FPEAK,
                            bbox=dict(boxstyle="round,pad=0.3",
                                      fc="white", ec="0.7", alpha=0.9))

    suptitle = fig.suptitle("connecting…", fontsize=11, y=0.97, ha="center")

    artists = (l5_raw, l5_clean, l5_peaks, l5_fft,
               l6_raw, l6_clean, l6_zcs, l6_fft, l6_peak,
               ann_f6, suptitle)
    axes_tuple = (ax_w5, ax_f5, ax_w6, ax_f6)
    return fig, axes_tuple, artists, t_axis


# ──────────────────────────────────────────────────────────────────────────────
# Animation update — fires every ~40 ms
# ──────────────────────────────────────────────────────────────────────────────
def make_update(buf, artists, t_axis, axes, window_s, mode, app_state):
    (l5_raw, l5_clean, l5_peaks, l5_fft,
     l6_raw, l6_clean, l6_zcs, l6_fft, l6_peak,
     ann_f6, suptitle) = artists
    ax_w5, ax_f5, ax_w6, ax_f6 = axes

    analysis_n = ANALYSIS_WINDOW_SEC * SAMPLE_RATE_HZ
    last_recompute = [0.0]
    cached = {"out_t": None, "out_r": None, "ylim5": None, "ylim6": None}

    def update(_frame):
        ch5_raw, ch6_raw = buf.snapshot()

        # ── Cheap update every frame: raw traces ────────────────────────────
        l5_raw.set_ydata(ch5_raw)
        l6_raw.set_ydata(ch6_raw)

        # ── BPM / FFT / peak overlay — recompute at 4 Hz only ───────────────
        now = time.time()
        if now - last_recompute[0] >= 0.25:
            last_recompute[0] = now

            # Use only the last ANALYSIS_WINDOW_SEC seconds for BPM estimation
            slice_ch5 = ch5_raw[-analysis_n:] if len(ch5_raw) >= analysis_n else ch5_raw
            slice_ch6 = ch6_raw[-analysis_n:] if len(ch6_raw) >= analysis_n else ch6_raw

            count_t, bpm_t, peaks, clean5, conf_t, freqs5, fft_mag5 = \
                estimate_thermistor(slice_ch5, SAMPLE_RATE_HZ, mode)
            (count_r, bpm_r, zcs, clean6,
             f_peak, conf_r, freqs6, fft_mag6) = \
                estimate_rubber(slice_ch6, SAMPLE_RATE_HZ, mode)

            bpm_fused, count_fused, (wt, wr), low_conf = \
                fuse_decisions(bpm_t, conf_t, count_t,
                               bpm_r, conf_r, count_r)

            cached["out_t"] = (count_t, bpm_t, conf_t)
            cached["out_r"] = (count_r, bpm_r, conf_r, f_peak)

            # Place the cleaned overlays at the right edge of the rolling
            # window (slice is the most recent samples).
            pad_full5 = np.full_like(ch5_raw, np.nan, dtype=float)
            pad_full5[-len(clean5):] = clean5
            l5_clean.set_ydata(pad_full5)

            pad_full6 = np.full_like(ch6_raw, np.nan, dtype=float)
            pad_full6[-len(clean6):] = clean6
            l6_clean.set_ydata(pad_full6)

            # Map analysis-window peak indices to rolling-window t coordinates
            offset = len(ch5_raw) - len(slice_ch5)
            if len(peaks):
                t_peaks = t_axis[peaks + offset]
                y_peaks = pad_full5[peaks + offset]
                l5_peaks.set_data(t_peaks, y_peaks)
            else:
                l5_peaks.set_data([], [])

            # ZC markers now sit on the cleaned signal (no narrowband overlay)
            offset6 = len(ch6_raw) - len(slice_ch6)
            if len(zcs):
                t_zcs = t_axis[zcs + offset6]
                y_zcs = pad_full6[zcs + offset6]
                l6_zcs.set_data(t_zcs, y_zcs)
            else:
                l6_zcs.set_data([], [])

            # FFT plots — CH5 from the thermistor, CH6 from the rubber
            l5_fft.set_data(freqs5, fft_mag5)
            l6_fft.set_data(freqs6, fft_mag6)
            ax_f5.set_ylim(0, max(fft_mag5.max() * 1.1, 1.0))
            ax_f6.set_ylim(0, max(fft_mag6.max() * 1.1, 1.0))

            if f_peak > 0:
                idx = int(np.argmin(np.abs(freqs6 - f_peak)))
                l6_peak.set_data([f_peak], [fft_mag6[idx]])
                ann_f6.xy = (f_peak, fft_mag6[idx])
                ann_f6.set_text(f"{f_peak:.3f} Hz  ·  {bpm_r:.1f} BPM")
            else:
                l6_peak.set_data([], [])
                ann_f6.set_text("")

            # Dynamic y-axis for waveform panels — keep the cleaned trace
            # visible regardless of the conditioning circuit's DC offset.
            for axw, raw, clean in ((ax_w5, ch5_raw, clean5),
                                    (ax_w6, ch6_raw, clean6)):
                rmin, rmax = raw.min(), raw.max()
                cmin, cmax = clean.min(), clean.max()
                lo = min(rmin, rmax - abs(cmax - cmin) * 1.2)
                hi = max(rmax, rmin + abs(cmax - cmin) * 1.2)
                pad = (hi - lo) * 0.05 + 5
                axw.set_ylim(lo - pad, hi + pad)

            # Status line in the suptitle
            mp = MODE_PARAMS[mode]
            rec_txt = ""
            if buf.recording:
                rec_dur = time.time() - buf.rec_start
                rec_txt = f"  ●  RECORDING {rec_dur:5.1f} s"
            wt_str = f"weights t={wt:.2f} r={wr:.2f}"
            warn = "  [low-confidence]" if low_conf else ""
            suptitle.set_text(
                f"{mp['label']} mode  ({mp['expected_range']})       "
                f"Thermistor: {bpm_t:5.1f} BPM (conf {conf_t:5.1f})    "
                f"Rubber: {bpm_r:5.1f} BPM (conf {conf_r:5.1f})    "
                f"FUSED: {bpm_fused:5.1f} BPM   {wt_str}{warn}{rec_txt}"
            )
            suptitle.set_color(COLOUR_REC if buf.recording else "black")

            # Auto-stop for exercise mode after EXERCISE_RECORD_SEC
            if (buf.recording and mode == "exercise"
                    and (time.time() - buf.rec_start) >= EXERCISE_RECORD_SEC):
                app_state["pending_stop"] = True

        # Process pending stop request from auto-timer or keyboard
        if app_state.get("pending_stop"):
            app_state["pending_stop"] = False
            do_stop_recording(buf, mode, app_state["output_dir"])

        return (l5_raw, l5_clean, l5_peaks, l5_fft,
                l6_raw, l6_clean, l6_zcs, l6_fft, l6_peak,
                ann_f6, suptitle)

    return update


# ──────────────────────────────────────────────────────────────────────────────
# Recording — save CSV and print summary
# ──────────────────────────────────────────────────────────────────────────────
def do_stop_recording(buf: SensorBuffer, mode: str, output_dir: Path) -> None:
    """
    Finalise the recording: pull the captured samples, run the same DSP +
    estimators on the FULL recorded duration (not just the rolling window),
    save a header-tagged CSV, and print a summary table.

    The CSV header contains every metric needed by analyze_session.py.
    The line `# ground_truth_count: ?` is intentionally left as a placeholder
    so you can edit it after the trial.
    """
    if not buf.recording and not buf.rec_ch5:
        return
    if buf.recording:
        rec5, rec6, t0 = buf.stop_recording()
    else:
        rec5 = np.array(buf.rec_ch5, dtype=float)
        rec6 = np.array(buf.rec_ch6, dtype=float)
        t0 = buf.rec_start

    duration_s = len(rec5) / SAMPLE_RATE_HZ
    if duration_s < 3.0:
        print(f"\n[recording too short: {duration_s:.1f} s — discarded]")
        return

    count_t, bpm_t, _, _, conf_t, _, _ = estimate_thermistor(rec5, SAMPLE_RATE_HZ, mode)
    (count_r, bpm_r, _, _, f_peak, conf_r, _, _) = \
        estimate_rubber(rec6, SAMPLE_RATE_HZ, mode)
    bpm_fused, count_fused, (wt, wr), low_conf = \
        fuse_decisions(bpm_t, conf_t, count_t, bpm_r, conf_r, count_r)

    stamp = datetime.fromtimestamp(t0).strftime("%Y%m%d_%H%M%S")
    fname = f"{mode}_{stamp}.csv"
    output_dir.mkdir(parents=True, exist_ok=True)
    fpath = output_dir / fname

    with open(fpath, "w", newline="") as f:
        f.write(f"# breath_analyzer.py recording\n")
        f.write(f"# mode: {mode}\n")
        f.write(f"# timestamp: {stamp}\n")
        f.write(f"# sample_rate_hz: {SAMPLE_RATE_HZ}\n")
        f.write(f"# duration_s: {duration_s:.3f}\n")
        f.write(f"# bpm_thermistor: {bpm_t:.3f}\n")
        f.write(f"# count_thermistor: {count_t}\n")
        f.write(f"# conf_thermistor: {conf_t:.3f}\n")
        f.write(f"# bpm_rubber: {bpm_r:.3f}\n")
        f.write(f"# count_rubber: {count_r}\n")
        f.write(f"# conf_rubber: {conf_r:.3f}\n")
        f.write(f"# f_peak_rubber_hz: {f_peak:.4f}\n")
        f.write(f"# bpm_fused: {bpm_fused:.3f}\n")
        f.write(f"# count_fused: {count_fused}\n")
        f.write(f"# weight_thermistor: {wt:.3f}\n")
        f.write(f"# weight_rubber: {wr:.3f}\n")
        f.write(f"# ground_truth_count: ?\n")
        f.write(f"# ground_truth_bpm: ?\n")
        f.write("sample_index,ch5,ch6\n")
        w = csv.writer(f)
        for i, (a, b) in enumerate(zip(rec5.astype(int), rec6.astype(int))):
            w.writerow([i, int(a), int(b)])

    bar = "─" * 60
    print(f"\n{bar}")
    print(f"  RECORDING STOPPED — {mode} mode")
    print(f"{bar}")
    print(f"  duration               : {duration_s:6.2f} s")
    print(f"  thermistor  count/BPM  : {count_t:3d}    {bpm_t:6.2f}   "
          f"(conf {conf_t:5.1f})")
    print(f"  rubber      count/BPM  : {count_r:3d}    {bpm_r:6.2f}   "
          f"(conf {conf_r:5.1f})  f_peak {f_peak:.3f} Hz")
    print(f"  fused       count/BPM  : {count_fused:3d}    {bpm_fused:6.2f}   "
          f"weights t={wt:.2f} r={wr:.5f}{'  [low confidence!]' if low_conf else ''}")
    print(f"  saved to               : {fpath}")
    print(f"  → edit `# ground_truth_count` / `# ground_truth_bpm` "
          f"in the header before running analyze_session.py")
    print(f"{bar}\n")


# ──────────────────────────────────────────────────────────────────────────────
# Keyboard handler
# ──────────────────────────────────────────────────────────────────────────────
def make_key_handler(buf, mode, app_state, fig):
    def on_key(event):
        key = (event.key or "").lower()
        if key == "r":
            if not buf.recording:
                buf.start_recording()
                hint = "press R again to stop" if mode == "rest" else \
                       f"auto-stop in {EXERCISE_RECORD_SEC} s"
                print(f"\n[record start — {mode} mode, {hint}]")
            else:
                app_state["pending_stop"] = True
        elif key == "q":
            plt.close(fig)
        elif key == "h":
            print("\nkeys:  R = start/stop recording   "
                  "Q = quit   H = help")
    return on_key


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live dual-sensor breath rate analyser (STM32 + thermistor + rubber)."
    )
    parser.add_argument("--port",   "-p", default="COM3",
                        help="serial port (e.g. COM3, /dev/ttyACM0)")
    parser.add_argument("--baud",   "-b", type=int, default=115200,
                        help="UART baud rate")
    parser.add_argument("--window", "-w", type=int, default=20,
                        help="rolling display window in seconds (default 20)")
    parser.add_argument("--mode",   "-m", choices=("rest", "exercise"),
                        default="rest",
                        help="rest (slow breathing) | exercise (fast breathing)")
    parser.add_argument("--output-dir", default=str(OUTPUT_DIR),
                        help="directory for recorded CSVs")
    args = parser.parse_args()

    # Select an interactive backend NOW (not at import-time) so this module
    # is importable from headless scripts like analyze_session.py.
    try:
        matplotlib.use("TkAgg")
    except Exception:
        try:
            matplotlib.use("Qt5Agg")
        except Exception:
            pass

    output_dir = Path(args.output_dir)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as exc:
        print(f"[ERROR] cannot open {args.port}: {exc}")
        sys.exit(1)

    print(f"opened {args.port} @ {args.baud}   mode = {args.mode}")
    print("keys:  R start/stop recording   Q quit\n")
    time.sleep(0.3)
    ser.reset_input_buffer()

    max_samples = args.window * SAMPLE_RATE_HZ
    buf = SensorBuffer(max_samples)

    stop_event = threading.Event()
    reader = threading.Thread(target=serial_reader,
                              args=(ser, buf, stop_event),
                              daemon=True)
    reader.start()

    fig, axes_tuple, artists, t_axis = build_figure(args.window, args.mode)
    fig.canvas.manager.set_window_title(
        f"Breath Analyser — {MODE_PARAMS[args.mode]['label']} mode")

    app_state = {"pending_stop": False, "output_dir": output_dir}

    update_fn = make_update(buf, artists, t_axis, axes_tuple,
                            args.window, args.mode, app_state)
    fig.canvas.mpl_connect("key_press_event",
                           make_key_handler(buf, args.mode, app_state, fig))

    # blit=False because the suptitle, FFT line, and y-limits change each
    # recompute pass; the saving in CPU from blit is not worth the artifact
    # bookkeeping it would force.
    ani = animation.FuncAnimation(
        fig, update_fn, interval=40, blit=False, cache_frame_data=False,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if buf.recording:
            do_stop_recording(buf, args.mode, output_dir)
        stop_event.set()
        reader.join(timeout=2.0)
        ser.close()
        print("\nconnection closed.")


if __name__ == "__main__":
    main()
