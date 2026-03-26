# Copyright (c) 2026 Nikolai Laptev (neparij)
# SPDX-License-Identifier: Zlib

"""Shared UGE file parsing for uge2midi / uge2xm."""
import struct
from collections import Counter

ROWS_PER_PATTERN = 64
CHANNELS = 4
EFFECT_NOTE_CUT = 0x0E
# ``effect-reference.md`` / ``utils.pas`` ``EffectToExplanation``: ``Cxy`` set channel volume (Param2 = y, 0..15)
# plus envelope mode from full param byte (Value). Handled in ``uge2xm`` pattern export.
EFFECT_SET_VOLUME = 0x0C
# ``constants.pas`` in hUGETracker: ``NO_NOTE = 90`` (cell ``___`` in driver macros; no new trigger).
UGE_NO_NOTE = 90


def uge_volume_15_to_xm_column_byte(vol15: int) -> int:
    """
    hUGE / DMG channel volume is **0..15**. XM / FT2 pattern column linear volume is **1..64**
    (stored as **0x10..0x4F**, see libxmp ``0x10..0x50`` branch: display value = byte − 0x0F).

    Linear map from 0..15 to 1..64 (``round(v * 64/15)``), **not** y mapped 1:1 (8/15 must not become 8/64).
    Returns **0** = leave column empty (no volume event on this row).
    """
    v = max(0, min(15, int(vol15)))
    if v <= 0:
        return 0
    xm01 = max(1, min(64, int(round(v * 64.0 / 15.0))))
    return 0x0F + xm01


def uge_j_column_to_xm_column_byte(j: int) -> int:
    """Tracker **J** column (stored as dword in .uge v6+); ``codegen.pas`` clamps 0..32."""
    j = int(j)
    if j <= 0:
        return 0
    if j <= 15:
        return uge_volume_15_to_xm_column_byte(j)
    xm01 = max(1, min(64, int(round(j * 64.0 / 32.0))))
    return 0x0F + xm01


def uge_pattern_row_xm_volume_byte(uge_eff: int, param: int, uge_cell_volume: int) -> int:
    """``C`` effect wins over **J** column on the same row (``utils.pas`` C semantics)."""
    if (int(uge_eff) & 0xFF) == EFFECT_SET_VOLUME:
        return uge_volume_15_to_xm_column_byte(param & 0x0F)
    if int(uge_cell_volume) > 0:
        return uge_j_column_to_xm_column_byte(uge_cell_volume)
    return 0

# ``tracker.pas`` ``UpdateBPMLabel``: VBlank rate when the GB timer is off.
VBLANK_HZ = 59.727500569606


def huge_timer_hz(timer_enabled: bool, timer_divider: int) -> float:
    """Interrupt rate used for row timing (``tracker.pas`` ``UpdateBPMLabel``)."""
    if timer_enabled:
        td = int(timer_divider) & 0xFF
        return 4096.0 / ((0xFF - td) + 1)
    return VBLANK_HZ


def xm_ft2_speed_bpm(ticks_per_row: int, timer_hz: float) -> tuple[int, int]:
    """
    FT2/XM row length equals ``speed * (2.5 / bpm)`` seconds per tick (OpenMPT/FT2).
    hUGE row length is ``ticks_per_row / timer_hz``. Pick ``speed`` and ``bpm`` so they match.
    If ``ticks_per_row`` exceeds 31 (XM speed limit), hold ``speed`` at 31 and lower ``bpm``.
    """
    tpr = max(1, min(int(ticks_per_row), 10_000))
    speed = min(31, max(1, tpr))
    bpm = round(2.5 * timer_hz * speed / tpr)
    bpm = max(32, min(255, bpm))
    return speed, bpm

# hUGETracker ``TInstrumentV2`` / ``TInstrumentV3`` are ``packed record``s. In FPC 3.2 default
# layout (same as hUGE builds): ``TInstrumentType`` and ``TSweepType`` are 4-byte enums;
# ``Duty`` is 1 byte directly before ``OutputLevel`` (no padding); ``TStepWidth`` is 4 bytes.
# See ``hugedatatypes.pas``; ``SizeOf(TInstrumentV2)`` = 310, ``SizeOf(TInstrumentV3)`` = 1385.

def u8(f):
    return struct.unpack("<B", f.read(1))[0]

def i8(f):
    return struct.unpack("<b", f.read(1))[0]

def u32(f):
    return struct.unpack("<I", f.read(4))[0]

def shortstring(f):
    """hUGE on-disk ShortString: 256 bytes (length in first byte, UTF-8-ish latin names)."""
    blob = f.read(256)
    length = blob[0]
    return blob[1 : 1 + length].decode("utf-8", errors="ignore")


def read_instrument_disk(f, version):
    """
    One instrument record: same binary layout as hUGETracker ``TInstrumentV2`` (version < 6)
    or ``TInstrumentV3`` (version >= 6). See ``hugedatatypes.pas``.
    """
    type_ = u32(f)
    name = shortstring(f)
    length = u32(f)
    length_enabled = u8(f) != 0
    initial_volume = u8(f)
    vol_sweep_direction = u32(f)
    vol_sweep_amount = u8(f)
    sweep_time = u32(f)
    sweep_inc_dec = u32(f)
    sweep_shift = u32(f)
    duty = u8(f)
    output_level = u32(f)
    waveform = u32(f)
    if version >= 6:
        counter_step = u32(f)
        subpattern_enabled = u8(f) != 0
        f.read(64 * 17)  # TPattern = 64 * SizeOf(TCellV2); ignore subpattern in tools
        shift_clock_freq = 0
        dividing_ratio = 0
        noise_macro = (0, 0, 0, 0, 0, 0)
    elif version >= 4:
        # TInstrumentV2 (+ NoiseMacro); SizeOf = 310 in FPC 3.2
        shift_clock_freq = u32(f)
        counter_step = u32(f)
        dividing_ratio = u32(f)
        noise_macro = struct.unpack("<6b", f.read(6))
        subpattern_enabled = False
    else:
        # TInstrumentV1 (no NoiseMacro); SizeOf = 304
        shift_clock_freq = u32(f)
        counter_step = u32(f)
        dividing_ratio = u32(f)
        noise_macro = (0, 0, 0, 0, 0, 0)
        subpattern_enabled = False
    return {
        "type_": type_,
        "name": name,
        "length": length,
        "length_enabled": length_enabled,
        "initial_volume": initial_volume,
        "vol_sweep_direction": vol_sweep_direction,
        "vol_sweep_amount": vol_sweep_amount,
        "sweep_time": sweep_time,
        "sweep_inc_dec": sweep_inc_dec,
        "sweep_shift": sweep_shift,
        "duty": duty,
        "output_level": output_level,
        "waveform": waveform,
        "shift_clock_freq": shift_clock_freq,
        "counter_step": counter_step,
        "dividing_ratio": dividing_ratio,
        "noise_macro": noise_macro,
        "subpattern_enabled": subpattern_enabled,
    }


def dominant_inst_notes(patterns, order, inst_lo=1, inst_hi=15):
    """Most common note code per instrument index (for CH4 frequency from ``NotesToFreqs``)."""
    c = {i: Counter() for i in range(inst_lo, inst_hi + 1)}
    for pid in order:
        rows = patterns.get(pid)
        if not rows:
            continue
        for note, inst, *_ in rows:
            if (
                note > 0
                and note not in (UGE_NO_NOTE, 91)
                and inst_lo <= inst <= inst_hi
            ):
                c[inst][note] += 1
    return {i: c[i].most_common(1)[0][0] for i in c if c[i]}


def dominant_inst_notes_merge(patterns, orders_list, inst_lo=1, inst_hi=15):
    """Merge note counts from several channel order lists (e.g. CH1+CH2 duty)."""
    c = {i: Counter() for i in range(inst_lo, inst_hi + 1)}
    for order in orders_list:
        for pid in order:
            rows = patterns.get(pid)
            if not rows:
                continue
            for note, inst, *_ in rows:
                if (
                    note > 0
                    and note not in (UGE_NO_NOTE, 91)
                    and inst_lo <= inst <= inst_hi
                ):
                    c[inst][note] += 1
    return {i: c[i].most_common(1)[0][0] for i in c if c[i]}

def skip_duty_instrument(f, version):
    read_instrument_disk(f, version)


def skip_wave_instrument(f, version):
    read_instrument_disk(f, version)


def skip_noise_instrument(f, version):
    read_instrument_disk(f, version)


def read_duty_instrument_name(f, version):
    u32(f)
    return shortstring(f)


def read_wave_instrument_name(f, version):
    u32(f)
    return shortstring(f)


def read_noise_instrument_name(f, version):
    u32(f)
    return shortstring(f)


def read_duty_instrument_full(f, version):
    d = read_instrument_disk(f, version)
    return {
        "type": d["type_"],
        "name": d["name"],
        "gb_len": d["length"],
        "len_enabled": 1 if d["length_enabled"] else 0,
        "init_vol": d["initial_volume"],
        "vol_dir": d["vol_sweep_direction"],
        "vol_spd": d["vol_sweep_amount"],
        "freq_time": d["sweep_time"],
        "freq_dir": d["sweep_inc_dec"],
        "freq_shift": d["sweep_shift"],
        "duty_raw": d["duty"],
    }


def read_wave_instrument_full(f, version):
    d = read_instrument_disk(f, version)
    return {
        "type": d["type_"],
        "name": d["name"],
        "gb_len": d["length"],
        "len_enabled": 1 if d["length_enabled"] else 0,
        "init_vol": d["initial_volume"],
        "vol_dir": d["vol_sweep_direction"],
        "vol_spd": d["vol_sweep_amount"],
        "freq_time": d["sweep_time"],
        "freq_dir": d["sweep_inc_dec"],
        "freq_shift": d["sweep_shift"],
        "wavetable_index": int(d["waveform"]) & 0x0F,
        "output_level": d["output_level"],
    }


def read_noise_instrument_full(f, version):
    d = read_instrument_disk(f, version)
    return {
        "type": d["type_"],
        "name": d["name"],
        "gb_len": d["length"],
        "len_enabled": 1 if d["length_enabled"] else 0,
        "init_vol": d["initial_volume"],
        "vol_dir": d["vol_sweep_direction"],
        "vol_spd": d["vol_sweep_amount"],
        "freq_time": d["sweep_time"],
        "freq_dir": d["sweep_inc_dec"],
        "freq_shift": d["sweep_shift"],
        "shift_clock_freq": d["shift_clock_freq"],
        "counter_step": d["counter_step"],
        "dividing_ratio": d["dividing_ratio"],
        "noise_macro": d["noise_macro"],
        "subpattern_enabled": d["subpattern_enabled"],
    }


def parse_uge_full(path):
    """Single read: patterns, orders, names, meta."""
    with open(path, "rb") as f:
        version = u32(f)
        song_name = shortstring(f)
        artist = shortstring(f)
        comment = shortstring(f)
        duty_instruments = [read_duty_instrument_full(f, version) for _ in range(15)]
        wave_instruments = [read_wave_instrument_full(f, version) for _ in range(15)]
        noise_instruments = [read_noise_instrument_full(f, version) for _ in range(15)]
        duty_names = [d["name"] for d in duty_instruments]
        wave_names = [d["name"] for d in wave_instruments]
        noise_names = [d["name"] for d in noise_instruments]
        wavetables = []
        for _ in range(16):
            wavetables.append(f.read(32))
            if version < 3:
                f.read(1)
        ticks_per_row = u32(f)
        timer_enabled = False
        timer_divider = 0
        if version >= 6:
            timer_enabled = u8(f) != 0
            timer_divider = u32(f)
        pattern_count = u32(f)
        patterns = {}
        for _ in range(pattern_count):
            index = u32(f)
            rows = []
            for _ in range(ROWS_PER_PATTERN):
                note = u32(f)
                instrument = u32(f)
                cell_volume = u32(f) if version >= 6 else 0
                effect = u32(f)
                param = u8(f)
                rows.append((note, instrument, cell_volume, effect, param))
            patterns[index] = rows
        orders = []
        for ch in range(CHANNELS):
            length = u32(f) - 1
            seq = []
            for _ in range(length):
                seq.append(u32(f))
            u32(f)
            orders.append(seq)
        return {
            "version": version,
            "song_name": song_name,
            "artist": artist,
            "comment": comment,
            "duty_names": duty_names,
            "wave_names": wave_names,
            "noise_names": noise_names,
            "duty_instruments": duty_instruments,
            "wave_instruments": wave_instruments,
            "noise_instruments": noise_instruments,
            "wavetables": wavetables,
            "ticks_per_row": ticks_per_row,
            "timer_enabled": timer_enabled,
            "timer_divider": timer_divider,
            "patterns": patterns,
            "orders": orders,
        }

def parse_uge(path):
    d = parse_uge_full(path)
    return d["ticks_per_row"], d["patterns"], d["orders"]
