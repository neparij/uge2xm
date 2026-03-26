# Copyright (c) 2026 Nikolai Laptev (neparij)
# SPDX-License-Identifier: Zlib

from __future__ import annotations

"""
PCM for UGE-to-XM aligned with hUGETracker ``sound.pas`` / ``instrumentpreview.pas``:

- CH1-3 use the same ``NotesToFreqs`` word as halt.gb preview (not CH4-only).
- CH1/CH2/CH4 amplitude: SameBoy ``vol[]`` table from ``sound.pas``.
- CH4: SameBoy ``NextLFSRBit`` + NR43 pacing; each output sample is the **mean DAC level
  over ``4194304/sample_rate`` CPU cycles** (same idea as ``SoundOutBits`` in ``sound.pas``),
  not the LFSR bit after ``floor(hz/sr)`` toggles.
- CH3: hUGE ``TWave`` file layout (32 bytes, one nibble per byte), NR32 cases like ``SoundUpdate``.

CH1 at 8363 Hz: a naive one-sample-per-period square aliases; pulse uses oversampling +
box filter. CH3 is **piecewise-constant** 4-bit DAC steps in ``sound.pas`` (no ``vol[]``);
averaging between steps (box on oversampled wave) **blurs** stair edges and changes
harmonics vs hUGE preview; default path is therefore **zero-order hold** only
(``oversample=1``).
"""
from typing import Any, Optional

from instruments.huge_constants import (
    HUGE_NOTE_TO_FREQ,
    SAMEBOY_PULSE_NOISE_VOL,
    ST_DOWN,
    ST_UP,
)

# DMG CPU clock; ``sound.pas`` SoundUpdate / CH4 timings are in these cycles.
_GB_CPU_CYCLES = 4194304
_GB_CPU_HZ = float(_GB_CPU_CYCLES)
# Envelope tick rate: ``sound.pas`` bumps ``envClk`` until ``8192*1024 div 64`` then runs NR12/NR22/NR42 steps.
_CH4_ENV_TICK_HZ = _GB_CPU_HZ / ((8192 * 1024) // 64)
# When NR42 ``Change`` is 0, CH4 volume never steps; without NR41 length the note sustains indefinitely.
# XM needs a long body + forward loop (like a wave loop). Cap file size (seconds of 8-bit mono at export rate).
_NOISE_UNBOUND_MAX_SEC = 16
# ``uge2xm`` ``compact`` profile: short forward looping grain (bytes ≈ sample count in XM).
_COMPACT_NOISE_BODY_SAMPLES = 1024
_COMPACT_NOISE_DECAY_RAW_CAP = 2048
_COMPACT_NOISE_ENV_LOOP_CAP = 1024
_COMPACT_NOISE_ENV_LOOP_FLOOR = 192
_COMPACT_NOISE_RISE_SUSTAIN = 1024

DUTY_CYCLES = (0.125, 0.25, 0.5, 0.75)


def duty_index_from_uge(raw: int) -> int:
    if raw <= 3:
        return raw & 3
    return (raw >> 6) & 3


def gb_square_hz(freq_word: int) -> float:
    n = max(0, min(2047, int(freq_word)))
    d = max(1, 2048 - n)
    return 131072.0 / d


def gb_wave_hz(freq_word: int) -> float:
    n = max(0, min(2047, int(freq_word)))
    d = max(1, 2048 - n)
    return 65536.0 / d


def _pulse_amplitudes_u8(vol_idx: int) -> tuple[int, int]:
    a = SAMEBOY_PULSE_NOISE_VOL[max(0, min(15, int(vol_idx)))]
    if a == 0:
        return 128, 128
    return 128 + a, 128 - a


def _box_downsample_channel(raw: bytes, factor: int, out_len: int) -> bytes:
    """Average ``factor`` consecutive samples; ``raw`` length must be ``out_len * factor``."""
    if factor <= 1:
        return raw
    out = bytearray(out_len)
    for i in range(out_len):
        base = i * factor
        s = 0
        for k in range(factor):
            s += raw[base + k]
        out[i] = s // factor
    return bytes(out)


def pulse_from_uge(
    duty_raw: int,
    init_vol: int,
    freq_word: int,
    sample_rate: int = 8363,
    min_period: int = 8,
    min_total_samples: int = 512,
    oversample: int = 8,
    *,
    compact_pcm: bool = False,
) -> tuple[bytes, int]:
    """
    One period of square at ``freq_word``; anti-alias via oversampling + box filter.
    """
    d = DUTY_CYCLES[duty_index_from_uge(duty_raw)]
    hz = gb_square_hz(freq_word)
    period = max(min_period, int(round(sample_rate / hz)))
    osamp = max(1, int(oversample))
    ph = period * osamp
    hi_n_h = max(1, int(round(ph * d)))
    if hi_n_h >= ph:
        hi_n_h = ph - 1
    hi_b, lo_b = _pulse_amplitudes_u8(init_vol)
    raw_h = bytearray(ph)
    for i in range(ph):
        raw_h[i] = hi_b if i < hi_n_h else lo_b
    chunk = _box_downsample_channel(bytes(raw_h), osamp, period)
    if compact_pcm:
        n_rep = 1
    else:
        n_rep = max(1, (min_total_samples + len(chunk) - 1) // len(chunk))
    return chunk * n_rep, len(chunk)


def _wave_nibble_after_nr32(nib: int, output_level: int) -> int:
    """``sound.pas`` CH3 NR32 output-level cases before ``(bit shl 4) - $80``."""
    nib &= 0xF
    level = int(output_level) & 3
    if level == 0:
        return 8
    if level == 1:
        return nib
    if level == 2:
        return 8 | (nib >> 1)
    return 0xC | (nib >> 2)


def _wave_nibble_twave(twave: bytes, stage: int) -> int:
    """
    hUGE ``TWave`` sample index 0..31: one nibble per byte (``utils.UnconvertWaveform`` /
    ``ConvertWaveform``; **not** 16-byte ``$FF30`` packing).
    """
    return int(twave[int(stage) & 31]) & 0xF


def _twave32_from_bytes(blob: bytes) -> bytes:
    """Normalize to 32-byte hUGE TWave: expand 16 B packed WRAM if needed, else pad/trim."""
    if len(blob) == 16:
        out = bytearray(32)
        for i in range(16):
            b = int(blob[i])
            out[i * 2] = (b >> 4) & 0xF
            out[i * 2 + 1] = b & 0xF
        return bytes(out)
    if len(blob) < 32:
        return bytes(blob) + bytes(32 - len(blob))
    return blob[:32]


def wave_pcm_from_uge(
    table32: bytes,
    output_level: int,
    freq_word: int,
    sample_rate: int = 8363,
    min_total_samples: int = 256,
    oversample: int = 1,
    *,
    compact_pcm: bool = False,
) -> tuple[bytes, int]:
    """
    One CH3 period from hUGE **TWave** (32 bytes, one nibble per byte). Same stage order as
    hardware after ``utils.ConvertWaveform``. NR32 handling matches ``sound.pas``.
    """
    table32 = _twave32_from_bytes(table32)
    hz = gb_wave_hz(freq_word)
    # Do not force a large minimum here: that stretches the loop vs ``65536/(2048-x)`` timing.
    n_out = max(2, int(round(sample_rate / hz)))
    osamp = max(1, int(oversample))
    n_h = n_out * osamp
    raw_h = bytearray(n_h)
    for i in range(n_h):
        pos = (i * 32.0 / n_h) % 32.0
        stage = min(31, int(pos))
        nib = _wave_nibble_twave(table32, stage)
        b = _wave_nibble_after_nr32(nib, output_level)
        raw_h[i] = (b << 4) & 0xFF
    out = _box_downsample_channel(bytes(raw_h), osamp, n_out)
    if compact_pcm:
        n_rep = 1
    else:
        n_rep = max(1, (min_total_samples + len(out) - 1) // len(out))
    loop = len(out)
    return out * n_rep, loop


def wavetable_to_pcm(table32: bytes) -> tuple[bytes, int]:
    """Backward-compatible: 100% level, hUGE C_4 (note 12) reference."""
    return wave_pcm_from_uge(table32, 1, HUGE_NOTE_TO_FREQ.get(12, 1046))


def noise_nr43_huge(freq_word: int, dividing_ratio: int, counter_step: int) -> int:
    """
    NR43 as in hUGE ``NoiseInstrumentToRegisters`` (file ``dividing_ratio``; preview forces 0).
    Export uses file value like XM path before.
    """
    fw = int(freq_word) & 0xFFFF
    shift = 0x0F - (fw >> 7)
    div = int(dividing_ratio) & 7
    seven = 1 if (int(counter_step) & 1) != 0 else 0
    return div | (seven << 3) | ((shift & 0x0F) << 4)


def ch4_period_cycles_from_nr43(nr43: int) -> int:
    """
    CPU cycles between ``NextLFSRBit`` calls; matches ``sound.pas`` after
    ``snd[4].Freq := (8192 * 1024) div ...`` (see ``sound.pas``).
    """
    d = nr43 & 7
    sh = (nr43 >> 4) & 0x0F
    if d == 0:
        inter = (512 * 1024 * 2) >> (sh + 1)
    elif d == 1:
        inter = (512 * 1024) >> (sh + 1)
    else:
        inter = ((512 * 1024) // d) >> (sh + 1)
    inter = max(1, inter)
    return max(1, (8192 * 1024) // inter)


def ch4_lfsr_hz_from_nr43(nr43: int) -> float:
    """CH4 LFSR step rate (steps/s) for diagnostics / callers that still want Hz."""
    return _GB_CPU_HZ / ch4_period_cycles_from_nr43(nr43)


def sameboy_lfsr_advance(lfsr: int, narrow: bool) -> tuple[int, int]:
    """``sound.pas`` ``NextLFSRBit`` state step; returns (new_state, bit)."""
    mask = 0x4040 if narrow else 0x4000
    new_high = (((lfsr ^ (lfsr >> 1)) ^ 1) & 1) != 0
    lfsr >>= 1
    if new_high:
        lfsr |= mask
    else:
        lfsr &= ~mask
    lfsr &= 0xFFFFFFFF
    return lfsr, lfsr & 1


def _ch4_sample_run(
    n: int,
    nr43: int,
    sample_rate: int,
    volume_fn,
) -> bytes:
    """
    Output one CH4 sample per interval ``1/sample_rate`` by **time-averaging** the DAC
    within that interval in **CPU cycles**, like hUGE ``SoundOutBits`` mixing many
    short ``SoundUpdate`` slices. Using only the LFSR state **after** all steps in the
    interval (old behaviour) biases fast noise and sounds choppy vs the tracker.
    """
    narrow = ((nr43 >> 3) & 1) != 0
    period = ch4_period_cycles_from_nr43(nr43)
    lfsr = 0
    freq4_clk = 0
    sr = max(1, int(sample_rate))
    c_acc = 0
    out = bytearray(n)
    for i in range(n):
        hi_b, lo_b = _pulse_amplitudes_u8(volume_fn(i))
        c_acc += _GB_CPU_CYCLES
        cycles_this = c_acc // sr
        c_acc %= sr
        rem = cycles_this
        acc = 0
        while rem > 0:
            bit = lfsr & 1
            level = hi_b if bit else lo_b
            space = period - freq4_clk
            if rem < space:
                acc += level * rem
                freq4_clk += rem
                rem = 0
            else:
                acc += level * space
                rem -= space
                freq4_clk = 0
                lfsr, _ = sameboy_lfsr_advance(lfsr, narrow)
        denom = max(1, cycles_this)
        out[i] = max(0, min(255, acc // denom))
    return bytes(out)


def _bake_xi_linear_volume_to_pcm_u8(
    loop_body: bytes,
    y0: int,
    x_end: int,
    sample_rate: int,
    bpm: float,
) -> bytes:
    """
    Linear amplitude ramp matching the XI envelope segment ``(0, y0)`` to ``(x_end, 0)`` in FT2 units
    (``y0/64`` max gain at start), over ``x_end`` ticks at ``2.5/bpm`` seconds per tick; same timing
    as ``noise_pcm_huge_xm``'s volume envelope. ``loop_body`` is CH4 noise at chip volume 15 (u8, ~128).
    """
    sr = max(1, int(sample_rate))
    bm = max(32.0, min(255.0, float(bpm)))
    xe = max(2, int(x_end))
    fade_len = max(2, int(round(float(xe) * sr * (2.5 / bm))))
    fade_len = min(fade_len, sr * 120)
    L = len(loop_body)
    if L == 0:
        return b""
    y0f = max(0.0, min(64.0, float(y0)))
    out = bytearray(fade_len)
    denom = float(fade_len - 1)
    for i in range(fade_len):
        t = i / denom
        gain = (y0f / 64.0) * (1.0 - t)
        s = float(loop_body[i % L])
        v = 128.0 + (s - 128.0) * gain
        out[i] = max(0, min(255, int(round(v))))
    return bytes(out)


def noise_pcm_huge_xm(
    ni: dict,
    note_code: int,
    sample_rate: int = 8363,
    xm_bpm: Optional[float] = None,
    *,
    compact_pcm: bool = False,
    bake_noise_volume_envelope: bool = False,
) -> tuple[bytes, int, int, Optional[dict[str, Any]], bool]:
    """
    CH4 sample + XI loop points from NR41/NR42/NR43 like ``sound.pas`` / DMG:

    - **Change (NR42 bits 0-2) == 0**: envelope never advances (``m_iram[$FF21] and 7 > 0`` gate).
      With length **off**, volume stays at **Start** for as long as the note runs: long PCM and a
      **forward loop over the full sample** (XM sustain body), same role as a pulse wave loop.
    - **Change > 0**: one step every ``Change`` ticks of the 32/s ``envClk`` block (same pattern as
      CH1/CH2 in ``SoundUpdate``). **Down**: decay to 0 (or XM loop + volume envelope when
      ``xm_bpm`` is set). **Up**: attack until 15, then **hold** at 15; loop is only the **hold**
      region (attack once, body loops).

    **Change = 0**: ``DrawEnvelope`` uses a flat line until **length**; not an NR42 ramp.

    When ``xm_bpm`` is passed, **decaying** noise uses a **forward-looped** NR43 body rendered at
    chip volume **15** (full swing), NR42 **Initial** as XM envelope **Y0** (``round(64*init/15)``),
    then a linear envelope **Y0 to 0** over a span derived from NR42 step rate vs FT2 ticks
    (``2.5/bpm``). **Fadeout** in the XI is **0**; loudness shape is entirely the volume envelope.

    If ``bake_noise_volume_envelope`` is true, **decaying** noise skips the XI envelope. With
    ``xm_bpm`` set, the vol-15 loop body is faded with the **same linear ramp and duration** as the
    XI would use (``(0, y0)`` to ``(x_end, 0)`` in tick time ``2.5/bpm``). Without ``xm_bpm``, bake falls
    back to **NR42+LFSR integration** like the non-OpenMPT path. For players (e.g. Maxmod) that
    mishandle XI envelopes.

    Returns ``(pcm, loop_start, loop_len, xm_instr_extra | None, envelope_baked_to_pcm)``.
    The last flag is true when decaying noise had its volume shape **in the sample** (baked linear
    fade or NR42-integrated body) so callers can e.g. peak-normalize without touching XI-driven noise.
    """
    freq = HUGE_NOTE_TO_FREQ.get(int(note_code), 1046)
    nr43 = noise_nr43_huge(
        freq,
        int(ni.get("dividing_ratio", 0)),
        int(ni.get("counter_step", 0)),
    )
    le = bool(int(ni.get("len_enabled", 0)))
    iv = max(0, min(15, int(ni.get("init_vol", 15))))
    vs = int(ni.get("vol_spd", 0)) & 7
    vdir = int(ni.get("vol_dir", ST_DOWN))
    increase = vdir == ST_UP
    ln = int(ni.get("gb_len", 0)) & 63

    sr = max(1, int(sample_rate))
    if le:
        len_samp = max(1, int(round(sr * (64 - ln) / 256.0)))
    else:
        len_samp = None

    env_period = max(1, int(round(sr * vs / _CH4_ENV_TICK_HZ))) if vs else 0

    narrow = ((nr43 >> 3) & 1) != 0
    period_cycles = ch4_period_cycles_from_nr43(nr43)

    def step_env(st: dict) -> None:
        if env_period <= 0:
            return
        st["ctr"] += 1
        if st["ctr"] >= env_period:
            st["ctr"] = 0
            if increase:
                st["v"] = min(15.0, st["v"] + 1.0)
            else:
                st["v"] = max(0.0, st["v"] - 1.0)

    def integrate_one(st: dict, lfsr: int, freq4_clk: int, c_acc: int) -> tuple[int, int, int, int]:
        vi = int(round(st["v"]))
        hi_b, lo_b = _pulse_amplitudes_u8(vi)
        c_acc += _GB_CPU_CYCLES
        cycles_this = c_acc // sr
        c_acc %= sr
        rem = cycles_this
        acc = 0
        while rem > 0:
            bit = lfsr & 1
            level = hi_b if bit else lo_b
            space = period_cycles - freq4_clk
            if rem < space:
                acc += level * rem
                freq4_clk += rem
                rem = 0
            else:
                acc += level * space
                rem -= space
                freq4_clk = 0
                lfsr, _ = sameboy_lfsr_advance(lfsr, narrow)
        denom = max(1, cycles_this)
        b = max(0, min(255, acc // denom))
        return b, lfsr, freq4_clk, c_acc

    # --- NR42 Change == 0: volume fixed ---
    if vs == 0:
        if iv <= 0:
            z = 32 if compact_pcm else 256
            return bytes([128]) * z, 0, 0, None, False
        if le:
            pcm = _ch4_sample_run(len_samp, nr43, sr, lambda _i: iv)
            return pcm, 0, 0, None, False
        if compact_pcm:
            n = max(256, min(_COMPACT_NOISE_BODY_SAMPLES, 0xFF0000))
        else:
            n = min(_NOISE_UNBOUND_MAX_SEC * sr, 0xFF0000)
            n = max(n, 4 * sr)
        pcm = _ch4_sample_run(n, nr43, sr, lambda _i: iv)
        return pcm, 0, len(pcm), None, False

    # --- decaying envelope ---
    if not increase:
        decay_budget = iv * env_period + env_period
        if len_samp is not None:
            decay_budget = min(decay_budget, len_samp)
        decay_budget = max(1, min(decay_budget, sr * 120, 0xFF0000))
        if compact_pcm:
            decay_budget = min(decay_budget, _COMPACT_NOISE_DECAY_RAW_CAP)

        def pcm_decay_integrated() -> bytes:
            st_i = {"v": float(iv), "ctr": 0}
            lfsr_i = 0
            fc_i = 0
            ca_i = 0
            raw_i = bytearray()
            for _ in range(decay_budget):
                step_env(st_i)
                b_i, lfsr_i, fc_i, ca_i = integrate_one(
                    st_i, lfsr_i, fc_i, ca_i
                )
                raw_i.append(b_i)
                if st_i["v"] <= 0:
                    break
            return bytes(raw_i)

        if xm_bpm is not None and iv > 0:
            bpm = max(32.0, min(255.0, float(xm_bpm)))
            tick_s = 2.5 / bpm
            samp_per_tick = max(1e-12, sr * tick_s)
            dx = max(1, int(round(env_period / samp_per_tick)))
            x_end = max(2, min(65535, int(round(iv * dx + dx * 0.25))))
            if len_samp is not None:
                cap_f = max(1, int(round(len_samp / samp_per_tick)))
                x_end = max(2, min(x_end, cap_f))
            # Loop must be long enough that LFSR output does not sound like a repeating tone.
            # Tie length to the same decay_budget we would use for baked PCM (NR42 steps * period),
            # with a floor ~200 ms of grain so high clocks do not get tiny loops.
            if compact_pcm:
                n_loop = max(
                    _COMPACT_NOISE_ENV_LOOP_FLOOR,
                    min(_COMPACT_NOISE_ENV_LOOP_CAP, decay_budget),
                )
            else:
                n_loop = max(int(sr * 0.2), min(65535, decay_budget))
            # Reference body at chip vol 15; NR42 **Initial** maps to XM envelope Y0 (0..64), decay in envelope.
            pcm_looped = _ch4_sample_run(n_loop, nr43, sr, lambda _i: 15)
            y0 = max(1, min(64, int(round(64 * iv / 15.0))))
            if bake_noise_volume_envelope:
                baked = _bake_xi_linear_volume_to_pcm_u8(
                    pcm_looped, y0, x_end, sr, bpm
                )
                return baked, 0, 0, None, True
            xm_extra: dict[str, Any] = {
                "vol_points": [(0, y0), (x_end, 0)],
                "vol_sus": 0,
                "vol_loop_start": 0,
                "vol_loop_end": 0,
                "vol_type": 0x01,
                "fadeout": 0,
            }
            return pcm_looped, 0, n_loop, xm_extra, False

        return (
            pcm_decay_integrated(),
            0,
            0,
            None,
            bool(bake_noise_volume_envelope),
        )

    # --- rising envelope: attack then sustain at 15 (loop = sustain only) ---
    if len_samp is not None:
        cap = len_samp
    else:
        cap = min(sr * 120, 0xFF0000)
        if compact_pcm:
            cap = min(cap, _COMPACT_NOISE_DECAY_RAW_CAP)
    st = {"v": float(iv), "ctr": 0}
    lfsr = 0
    freq4_clk = 0
    c_acc = 0
    rise = bytearray()

    if iv < 15:
        while st["v"] < 15.0 and len(rise) < cap:
            step_env(st)
            b, lfsr, freq4_clk, c_acc = integrate_one(st, lfsr, freq4_clk, c_acc)
            rise.append(b)
            if st["v"] >= 15.0:
                break
    rise_len = len(rise)

    if len_samp is not None:
        sustain_n = max(0, len_samp - rise_len)
    else:
        if compact_pcm:
            sustain_n = min(_COMPACT_NOISE_RISE_SUSTAIN, 0xFF0000)
        else:
            sustain_n = min(max(4 * sr, 4096), 0xFF0000)

    if sustain_n <= 0:
        return bytes(rise), 0, 0, None, False
    sustain = _ch4_sample_run(sustain_n, nr43, sr, lambda _i: 15)
    full = bytes(rise) + sustain
    if rise_len == 0:
        return full, 0, sustain_n, None, False
    return full, rise_len, sustain_n, None, False


def noise_lfsr_pcm(init_vol: int, seven_bit: bool, length: int = 4096) -> tuple[bytes, int]:
    nr43 = noise_nr43_huge(1046, 0, 1 if seven_bit else 0)
    pcm = _ch4_sample_run(length, nr43, 8363, lambda _i: init_vol)
    return pcm, min(length, 4096)
