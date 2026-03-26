#!/usr/bin/env python3
# Copyright (c) 2026 Nikolai Laptev (neparij)
# SPDX-License-Identifier: Zlib

"""Convert hUGETracker .uge to FastTracker II .xm (4 channels, merged pattern order.

Instruments: 45 slots (15 pulse + 15 wave + 15 noise). Samples are built from UGE data
(duty/volume, wavetable RAM, noise mode) via ``instruments.gb_chip_pcm``; generic
placeholders are used only if parsing yields no instrument list.

Tempo: XM ``speed``/``bpm`` are chosen so each pattern row lasts as long as in hUGE:
``ticks_per_row / timer_hz`` (``tracker.pas`` ``UpdateBPMLabel``, VBlank or GB timer).
That FT2 ``bpm`` is not the same number as the tracker status ``~xx BPM`` (there one beat
is **4 rows**).

Optional ``--c-envelope-volslide``: ``C**`` with hardware envelope **down/up** (see
``utils.pas``) gets a **multi-row linear ramp** in the XM **volume column** (FT2 0x10..0x50),
row-aligned to DMG envelope time ``steps * (P+1) / 64`` s; smoother than a single **Axy**
slide; off by default.

PCM is **compact** (one pulse/wave period in the loop; short noise bodies) so ``.xm`` stays small.
Embedded sample format is chosen with ``--sample-format`` (default **s8**): 8-bit **u8** vs **s8**
wire for OpenMPT, or 16-bit (**s16**, **u16** alias) with XM flag ``0x10``. All 45 instruments use
the same format per run.

Optional ``--bake-noise-envelope`` ``raw`` / ``normalized`` (or flag alone = ``raw``): for decaying CH4,
bake volume into PCM instead of XI; ``normalized`` peak-normalizes only that baked noise.

Optional ``--remove-unused``: drop XM instruments never referenced on a note in the song and
renumber the pattern instrument column (fewer embedded samples, smaller file).

XM **module header** uses a compact order table (``headersz = 20 + song_len``) and patterns are
written in **FT2 packed** form (``0x80`` empty slot, or mask byte + fields), matching libxmp/OpenMPT.
"""
import argparse
import math
import os
import struct
from typing import Any, Mapping, Optional

from uge_common import (
    parse_uge_full,
    ROWS_PER_PATTERN,
    CHANNELS,
    EFFECT_NOTE_CUT,
    EFFECT_SET_VOLUME,
    UGE_NO_NOTE,
    dominant_inst_notes,
    huge_timer_hz,
    uge_pattern_row_xm_volume_byte,
    uge_volume_15_to_xm_column_byte,
    xm_ft2_speed_bpm,
)
from instruments.huge_constants import HUGE_NOTE_TO_FREQ, ST_UP
from instruments.gb_chip_pcm import noise_pcm_huge_xm, pulse_from_uge, wave_pcm_from_uge
from instruments.xm_placeholders import (
    XM_SAMPLE_FORMATS,
    duty_pulse_pcm,
    wave_triangle_pcm,
    noise_pcm,
    xm_sample_pack_chip_pcm,
    PULSE_PERIOD,
    WAVE_PERIOD,
)

XM_VERSION = 0x0104
# Offset 60: ``headersz`` = 4 (self) + 16 (song metadata through BPM) + ``song_len`` order bytes.
# libxmp reads ``song_len_order = headersz - 20`` bytes of pattern order (not always 256).
CHANNELS_XM = 4
# FT2 XM pattern packing (``libxmp`` ``load_xm_pattern``): high bit set, then mask + optional fields.
_XM_PACK = 0x80
_XM_NOTE = 0x01
_XM_INST = 0x02
_XM_VOL = 0x04
_XM_FXT = 0x08
_XM_FXP = 0x10
# UGE: 15 pulse (ch1+ch2) + 15 wave (ch3) + 15 noise (ch4) => 45 XM slots so each channel keeps correct sample bank.
NUM_META_INSTR = 45
# FT2: instrument size = bytes from start of inst (incl. size dword) through end of 208-byte chunk + reserved; NOT incl. sample headers/PCM.
# libxmp: read 33 + 208, then seek xih.size - 241 (= +22). Then 40-byte sample header(s), then delta PCM.
XM_INSTRUMENT_BYTES_BEFORE_SAMPLE_HEADERS = 263
XM_XI_PADDING = XM_INSTRUMENT_BYTES_BEFORE_SAMPLE_HEADERS - 33 - 208  # 22
# Pulse/wave samples are rendered at this hUGE note frequency (``constants.pas`` C_4 = 12).
# Pattern note index follows FT2 (1 = C-0). OpenMPT shows note **names** one octave higher
# than hUGE-style labels unless the pattern is **12 semitones lower** and each sample's
# **RelativeNote** is **+12** (XM). Noise uses a **per-slot** relative (see ``xm_noise_relative_note``).
XM_SAMPLE_REF_HUGE_NOTE = 12
XM_SAMPLE_RELATIVE_NOTE = 12
def normalize_chip_pcm_u8_peak(pcm_u8: bytes) -> bytes:
    """Peak-normalize chip-style mono u8 (128 = DC): scale signed samples so max |s| reaches 127."""
    if not pcm_u8:
        return pcm_u8
    signed = [int(b) - 128 for b in pcm_u8]
    peak = max((abs(s) for s in signed), default=0)
    if peak == 0:
        return pcm_u8
    scale = 127.0 / float(peak)
    out = bytearray(len(pcm_u8))
    for i, s in enumerate(signed):
        v = int(round(s * scale))
        v = max(-128, min(127, v))
        out[i] = (v + 128) & 0xFF
    return bytes(out)


def xm_noise_relative_note(note_code_huge: int) -> int:
    """
    XI sample **Relative note** (signed, stored as byte) for CH4 noise.

    Pulse/wave PCM is built at ``XM_SAMPLE_REF_HUGE_NOTE`` (C_4 = 12); pattern uses ``25+n`` and
    ``RelativeNote = +12`` so effective row matches the ``37+n`` chip map (see ``uge_note_to_xm``).

    Noise PCM is built at NR43 from **dominant** ``note_code_huge``. Flat **+12** like pulse still
    biases high in OpenMPT. A **full** root correction would be ``24 - note_c``; that proved a bit
    strong, so we apply **half** of ``(12 - note_c)``:

    ``Rel = 12 + round(0.5 * (12 - note_c))``  (``note_c == 12`` gives **+12**; ``note_c == 60`` gives **-12**).
    """
    n = max(0, min(71, int(note_code_huge)))
    half = int(round(0.5 * float(XM_SAMPLE_REF_HUGE_NOTE - n)))
    r = XM_SAMPLE_RELATIVE_NOTE + half
    return max(-128, min(127, int(r)))


# FT2/XM sample-header volume is **0..64** (40-byte subsample header). Older code wrote byte 0x64
# (= decimal **100**); out of spec; players clamp or scale oddly vs hUGE. Default below is below max
# so stacked CH1-CH4 in OpenMPT rarely hard-clips.
XM_SAMPLE_DEFAULT_VOLUME = 64


def _pad_str(s: str, n: int) -> bytes:
    b = s.encode("latin-1", errors="replace")[: n]
    return b + bytes(n - len(b))


def uge_note_to_xm(note_raw: int) -> int:
    """Map hUGE note code 0..71 (C_3..B_8) to FT2 pattern note 1..96 (1 = C-0).

    Base FT2 index for hUGE note ``n`` is ``37 + n`` (``C_3`` is 37). **Subtract 12** so
    OpenMPT's pattern column names align with hUGE (``B-4`` stays ``B-4``); keep pitch using
    ``XM_SAMPLE_RELATIVE_NOTE = +12`` on instruments.
    """
    if note_raw < 0 or note_raw in (UGE_NO_NOTE, 91):
        return 0
    return max(1, min(96, 25 + int(note_raw)))


def xm_instrument_index(channel: int, uge_inst: int) -> int:
    """UGE instrument column 1..15 -> XM instrument 1..45."""
    if uge_inst <= 0:
        return 0
    if channel <= 1:
        return uge_inst
    if channel == 2:
        return 15 + uge_inst
    return 30 + uge_inst


def collect_used_xm_instrument_indices(
    patterns: dict,
    orders: list,
    order_len: int,
) -> set[int]:
    """XM instrument numbers (1..45) that appear on a triggered note in any channel/order."""
    used: set[int] = set()
    for pos in range(order_len):
        for row in range(ROWS_PER_PATTERN):
            for ch in range(CHANNELS_XM):
                pid = orders[ch][pos]
                prow = patterns.get(pid)
                if not prow:
                    continue
                note, uge_inst, _, _, _ = prow[row]
                if note == UGE_NO_NOTE or note == 91:
                    continue
                xn = uge_note_to_xm(note)
                if not xn:
                    continue
                xi = xm_instrument_index(ch, uge_inst)
                if xi > 0:
                    used.add(xi)
    return used


def _c_envelope_decay_span(
    param: int, ticks_per_row: int, timer_hz: float
) -> Optional[tuple[float, float, float]]:
    """
    If ``C`` byte selects envelope **down** or **up** (``utils.pas``), return
    ``(v0_15, v1_15, duration_s)`` for a linear ramp in DMG volume 0..15; else ``None``.
    ``duration_s`` is total time for the envelope phase on hardware (approx.).
    """
    v = int(param) & 0xFF
    if v < 16 or (128 <= v < 144):
        return None
    ph = (v >> 4) & 0x0F
    pl = float(v & 0x0F)
    if 16 <= v < 128:
        if pl <= 0:
            return None
        p = ph & 0x07
        div = float(p + 1 if p != 0 else 8)
        total_s = pl * div / 64.0
        return pl, 0.0, total_s
    ru = int(ph) - 8
    if ru <= 0:
        ru = 1
    ru = min(7, ru)
    div = float(ru + 1)
    if pl >= 15:
        return None
    total_s = (15.0 - pl) * div / 64.0
    return pl, 15.0, total_s


def _apply_c_envelope_volume_ramp(
    cells: list[list[dict[str, Any]]],
    ticks_per_row: int,
    timer_hz: float,
) -> None:
    """
    DMG envelope changes volume in ``~1/64 s`` steps. Approximate **down/up** ``C`` with a
    linear **volume column** (FT2 linear 1..64) across multiple rows, duration matched to
    ``duration_s`` from hardware.
    """
    row_s = float(max(1, int(ticks_per_row))) / max(0.001, float(timer_hz))
    for ch in range(CHANNELS_XM):
        decay: Optional[dict[str, Any]] = None
        for row in range(ROWS_PER_PATTERN):
            c = cells[row][ch]
            note = int(c["note"])
            uge_eff = int(c["uge_eff"]) & 0xFF
            param = int(c["param"]) & 0xFF
            uge_cell_vol = int(c["uge_cell_vol"])

            if decay is not None:
                if row >= int(decay["end_row"]):
                    decay = None
                elif note == 91:
                    decay = None
                elif note != UGE_NO_NOTE:
                    decay = None
                elif uge_cell_vol > 0:
                    decay = None
                elif uge_eff == EFFECT_SET_VOLUME and row > int(decay["start_row"]):
                    decay = None

            if (
                decay is not None
                and row > int(decay["start_row"])
                and row < int(decay["end_row"])
                and note == UGE_NO_NOTE
                and uge_cell_vol == 0
                and uge_eff != EFFECT_SET_VOLUME
            ):
                nrows = int(decay["n_rows"])
                k = row - int(decay["start_row"])
                if nrows <= 1:
                    t = 1.0
                else:
                    t = k / float(nrows - 1)
                v0 = float(decay["v0"])
                v1 = float(decay["v1"])
                v15 = v0 * (1.0 - t) + v1 * t
                v15i = int(round(v15))
                v15i = max(0, min(15, v15i))
                # ``0`` in the XM volume byte = "no column event"; previous level would stick.
                if v15i <= 0:
                    xv = 0x10  # quietest explicit linear vol (1/64)
                else:
                    xv = uge_volume_15_to_xm_column_byte(v15i)
                c["xv"] = xv

            if uge_eff == EFFECT_SET_VOLUME:
                span = _c_envelope_decay_span(param, ticks_per_row, timer_hz)
                if span is not None:
                    v0, v1, total_s = span
                    n_rows = max(1, int(math.ceil(total_s / row_s)))
                    decay = {
                        "start_row": row,
                        "end_row": row + n_rows,
                        "n_rows": n_rows,
                        "v0": v0,
                        "v1": v1,
                    }


def map_uge_effect_to_xm(
    uge_eff: int,
    param: int,
    uge_ticks_per_row: int,
    xm_speed: int,
) -> tuple[int, int]:
    if uge_eff == 0x0F:
        return 0x0F, param & 0xFF
    if uge_eff == EFFECT_NOTE_CUT:
        # ``effect-reference.md`` Exx: cut after xx ticks only if xx < tempo (ticks/row);
        # if xx >= tempo the note plays the full row. Params are a full byte (``utils.pas``).
        # XM ECx (ft2 extended): volume 0 after x ticks; ignored if x >= row speed (OpenMPT).
        tpr = max(1, int(uge_ticks_per_row))
        spd = max(1, min(31, int(xm_speed)))
        p = int(param) & 0xFF
        if p >= tpr:
            return 0, 0
        tick_xm = int(round(p * spd / tpr))
        tick_xm = max(0, min(tick_xm, spd - 1))
        tick_ec = min(tick_xm, 15)
        return 0x0E, 0xC0 | tick_ec
    # ``C`` is absorbed into XM **volume column** (1..64 from y/15); FT2 effect C is not the same.
    if uge_eff == EFFECT_SET_VOLUME:
        return 0, 0
    if uge_eff == 0x08:
        return 0x08, param & 0xFF
    if uge_eff == 0x0B:
        # hUGEDriver ``fx_pos_jump``: param 0 stores ``next_order`` as 0, so the break
        # path takes ``.neworder`` and **advances** to the next order (same as finishing
        # the pattern). Non-zero param P loads order index ``2*(P-1)``, jump to order
        # ``P-1`` (0-based). FT2 **B00** jumps to playlist row 0 instead, so map **B00**
        # to **D00** (pattern break, next order, row 0); **B01**.. to **B00**..
        p = int(param) & 0xFF
        if p == 0:
            return 0x0D, 0
        return 0x0B, (p - 1) & 0xFF
    if uge_eff == 0x0D:
        return 0x0D, param & 0xFF
    return 0, 0


def pack_xm_pattern_cell(note: int, inst: int, vol: int, fxt: int, fxp: int) -> bytes:
    """One channel column in FT2 XM packed pattern stream (not legacy 5-byte unpacked)."""
    note &= 0xFF
    inst &= 0xFF
    vol &= 0xFF
    fxt &= 0xFF
    fxp &= 0xFF
    if note == 0 and inst == 0 and vol == 0 and fxt == 0 and fxp == 0:
        return bytes([_XM_PACK])
    mask = _XM_PACK
    parts: list[int] = []
    if note:
        mask |= _XM_NOTE
        parts.append(note)
    if inst:
        mask |= _XM_INST
        parts.append(inst)
    if vol:
        mask |= _XM_VOL
        parts.append(vol)
    if fxt or fxp:
        mask |= _XM_FXT | _XM_FXP
        parts.append(fxt)
        parts.append(fxp)
    return bytes([mask]) + bytes(parts)


def _pack_one_xjm_cell(
    ch: int,
    c: dict[str, Any],
    *,
    xm_instr_remap: Optional[dict[int, int]] = None,
) -> bytes:
    """Serialize one merged track cell after optional C-envelope ramp pass."""
    note = int(c["note"])
    ui = int(c["uge_inst"])
    xv = int(c["xv"])
    fxt = int(c["fxt"])
    fxp = int(c["fxp"])

    if note == UGE_NO_NOTE:
        if fxt or fxp or xv:
            return pack_xm_pattern_cell(0, 0, xv, fxt, fxp)
        return pack_xm_pattern_cell(0, 0, 0, 0, 0)
    if note == 91:
        return pack_xm_pattern_cell(97, 0, xv, fxt, fxp)
    xn = uge_note_to_xm(note)
    if xn:
        xi = xm_instrument_index(ch, ui)
        if xm_instr_remap is not None and xi:
            xi = xm_instr_remap[xi]
        return pack_xm_pattern_cell(xn, xi, xv, fxt, fxp)
    if fxt or fxp or xv:
        return pack_xm_pattern_cell(0, 0, xv, fxt, fxp)
    return pack_xm_pattern_cell(0, 0, 0, 0, 0)


def build_pattern_data(
    patterns: dict,
    orders: list,
    pos: int,
    ticks_per_row: int,
    xm_speed: int,
    *,
    c_envelope_volslide: bool = False,
    timer_hz: float = 59.7275,
    xm_instr_remap: Optional[dict[int, int]] = None,
) -> bytes:
    """One XM pattern: 64 rows by 4 channels."""
    cells: list[list[dict[str, Any]]] = []
    for row in range(ROWS_PER_PATTERN):
        row_cells: list[dict[str, Any]] = []
        for ch in range(CHANNELS_XM):
            pid = orders[ch][pos]
            prow = patterns.get(pid)
            if not prow:
                row_cells.append(
                    {
                        "ch": ch,
                        "note": UGE_NO_NOTE,
                        "uge_inst": 0,
                        "uge_cell_vol": 0,
                        "uge_eff": 0,
                        "param": 0,
                        "xv": 0,
                        "fxt": 0,
                        "fxp": 0,
                    }
                )
                continue
            note, uge_inst, uge_cell_vol, uge_eff, param = prow[row]
            fxt, fxp = map_uge_effect_to_xm(uge_eff, param, ticks_per_row, xm_speed)
            xv = uge_pattern_row_xm_volume_byte(uge_eff, param, uge_cell_vol)
            row_cells.append(
                {
                    "ch": ch,
                    "note": int(note),
                    "uge_inst": int(uge_inst),
                    "uge_cell_vol": int(uge_cell_vol),
                    "uge_eff": int(uge_eff),
                    "param": int(param),
                    "xv": int(xv),
                    "fxt": int(fxt),
                    "fxp": int(fxp),
                }
            )
        cells.append(row_cells)

    if c_envelope_volslide:
        _apply_c_envelope_volume_ramp(cells, ticks_per_row, timer_hz)

    out = bytearray()
    for row in range(ROWS_PER_PATTERN):
        for ch in range(CHANNELS_XM):
            out.extend(
                _pack_one_xjm_cell(
                    ch, cells[row][ch], xm_instr_remap=xm_instr_remap
                )
            )
    return bytes(out)


def write_xm_module_header(
    f,
    title: str,
    song_len: int,
    num_patterns: int,
    num_instruments: int,
    tempo: int,
    bpm: int,
):
    f.write(b"Extended Module: ")
    f.write(_pad_str(title, 20))
    f.write(bytes([0x1A]))
    f.write(_pad_str("uge2xm", 20))
    f.write(XM_VERSION.to_bytes(2, "little"))
    header_size = 4 + 16 + song_len
    f.write(header_size.to_bytes(4, "little"))
    f.write(song_len.to_bytes(2, "little"))
    f.write((0).to_bytes(2, "little"))
    f.write(CHANNELS_XM.to_bytes(2, "little"))
    f.write(num_patterns.to_bytes(2, "little"))
    f.write(num_instruments.to_bytes(2, "little"))
    f.write((1).to_bytes(2, "little"))  # linear frequency table
    t = max(1, min(31, tempo))
    f.write(t.to_bytes(2, "little"))
    f.write(max(32, min(255, bpm)).to_bytes(2, "little"))
    order = bytearray(song_len)
    for i in range(song_len):
        order[i] = i
    f.write(order)


def write_pattern(f, packed: bytes):
    hdr_len = 9
    f.write(hdr_len.to_bytes(4, "little"))
    f.write(bytes([0]))
    f.write(ROWS_PER_PATTERN.to_bytes(2, "little"))
    f.write(len(packed).to_bytes(2, "little"))
    f.write(packed)


def pack_xm_instrument_ext208(
    *,
    vol_points: list[tuple[int, int]],
    vol_sus: int = 0,
    vol_loop_start: int = 0,
    vol_loop_end: int = 0,
    vol_type: int = 1,
    fadeout: int = 256,
) -> bytes:
    """
    FT2 208-byte extended instrument block after ``sample_header_size`` (libxmp layout).
    Volume envelope Y uses 0..64. Fadeout 0..0xFFF (0 = no post-keyoff fade; shape from envelope only).
    """
    if len(vol_points) > 12:
        vol_points = vol_points[:12]
    buf = bytearray(208)
    # 0:96 key-to-sample map (one sample index for note k+12); single sample: all 0.
    # 96:96+48 twelve (x,y) pairs as uint16 LE for volume envelope.
    for j, (x, y) in enumerate(vol_points):
        struct.pack_into("<HH", buf, 96 + j * 4, min(65535, max(0, x)), min(64, max(0, y)))
    off = 192
    buf[off] = len(vol_points)
    buf[off + 1] = 0  # pan points
    buf[off + 2] = vol_sus & 0xFF
    buf[off + 3] = vol_loop_start & 0xFF
    buf[off + 4] = vol_loop_end & 0xFF
    buf[off + 5] = 0
    buf[off + 6] = 0
    buf[off + 7] = 0
    buf[off + 8] = vol_type & 0xFF
    buf[off + 9] = 0  # pan type
    buf[off + 10] = 0  # vibrato type/sweep/depth/rate
    buf[off + 11] = 0
    buf[off + 12] = 0
    buf[off + 13] = 0
    struct.pack_into("<H", buf, 206, max(0, min(0xFFF, int(fadeout))))
    return bytes(buf)


def write_instrument(
    f,
    name: str,
    pcm_delta: bytes,
    loop_start: int = 0,
    loop_length: int = 0,
    relative_note: int = XM_SAMPLE_RELATIVE_NOTE,
    ext208: Optional[bytes] = None,
    sample_volume: int = XM_SAMPLE_DEFAULT_VOLUME,
    *,
    sample_bits: int = 8,
):
    """``loop_length``/``loop_start`` are **bytes** in the file (16-bit mono uses 2x vs frame counts).

    ``sample_bits`` 8 or 16: FT2 delta on the payload from ``xm_sample_pack_chip_pcm`` / ``--sample-format``.
    """
    sb = 16 if int(sample_bits) == 16 else 8
    sv = max(0, min(64, int(sample_volume)))
    f.write(XM_INSTRUMENT_BYTES_BEFORE_SAMPLE_HEADERS.to_bytes(4, "little"))
    f.write(_pad_str(name[:22], 22))
    f.write(bytes([0]))
    f.write((1).to_bytes(2, "little"))
    f.write((40).to_bytes(4, "little"))
    if ext208 is None:
        # FT2/OpenMPT expect a real XI-style block: all-zero 208 bytes leave vol_fadeout=0 at
        # bytes 206-207 of XMInstrument, which is atypical (FT2 default fadeout is 256).
        ext208 = pack_xm_instrument_ext208(
            vol_points=[],
            vol_type=0,
            fadeout=256,
        )
    elif len(ext208) != 208:
        raise ValueError("ext208 must be 208 bytes")
    f.write(ext208)
    f.write(bytes(XM_XI_PADDING))
    slen = len(pcm_delta)
    if sb == 16:
        sample_type = 0x10 | (0x01 if loop_length > 0 else 0x00)
    else:
        sample_type = 0x01 if loop_length > 0 else 0x00
    f.write(slen.to_bytes(4, "little"))
    f.write(loop_start.to_bytes(4, "little"))
    f.write(loop_length.to_bytes(4, "little"))
    f.write(bytes([sv]))
    f.write(bytes([0]))
    f.write(bytes([sample_type]))
    f.write(bytes([0x80]))
    rn = int(relative_note) & 0xFF
    f.write(bytes([rn]))
    f.write(bytes([0]))
    f.write(_pad_str("", 22))
    f.write(pcm_delta)


def build_instrument_names(duty_names, wave_names, noise_names) -> list[str]:
    names = []
    for i, n in enumerate(duty_names):
        names.append(f"D{i+1:02d} {n}"[:21])
    for i, n in enumerate(wave_names):
        names.append(f"W{i+1:02d} {n}"[:21])
    for i, n in enumerate(noise_names):
        names.append(f"N{i+1:02d} {n}"[:21])
    return names


def uge_to_xm(
    uge_path: str,
    xm_path: str,
    bpm_override: Optional[int] = None,
    verbose: bool = False,
    c_envelope_volslide: bool = False,
    *,
    bake_noise_envelope: Optional[str] = None,
    remove_unused: bool = False,
    sample_format: str = "s8",
):
    d = parse_uge_full(uge_path)
    patterns = d["patterns"]
    orders = d["orders"]
    ticks_per_row = max(1, d["ticks_per_row"])
    timer_hz = huge_timer_hz(
        bool(d.get("timer_enabled")),
        int(d.get("timer_divider", 0)),
    )
    xm_speed, xm_bpm = xm_ft2_speed_bpm(ticks_per_row, timer_hz)
    if bpm_override is not None:
        xm_bpm = max(32, min(255, int(bpm_override)))
    xm_svol = max(0, min(64, int(XM_SAMPLE_DEFAULT_VOLUME)))
    # hUGE UI: 4 rows = one beat (``tracker.pas`` UpdateBPMLabel)
    huge_beat_bpm = round(60.0 * timer_hz / (4.0 * ticks_per_row), 2)
    duty_names = d["duty_names"]
    wave_names = d["wave_names"]
    noise_names = d["noise_names"]
    title = d["song_name"] or "untitled"

    order_len = min(256, max(len(orders[ch]) for ch in range(CHANNELS)))
    padded_orders = []
    for ch in range(CHANNELS):
        seq = list(orders[ch])
        if len(seq) < order_len:
            pad = seq[-1] if seq else 0
            seq.extend([pad] * (order_len - len(seq)))
        padded_orders.append(seq)
    orders = padded_orders

    instr_names = build_instrument_names(duty_names, wave_names, noise_names)
    duty_inst = d.get("duty_instruments") or []
    wave_inst = d.get("wave_instruments") or []
    noise_inst = d.get("noise_instruments") or []
    wavetables = d.get("wavetables") or []
    noise_note_pref = dominant_inst_notes(patterns, orders[3])
    fw_chip = HUGE_NOTE_TO_FREQ.get(XM_SAMPLE_REF_HUGE_NOTE, 1046)
    bake_vol = bake_noise_envelope is not None

    sf = sample_format.strip().lower()
    if sf not in XM_SAMPLE_FORMATS:
        raise ValueError(
            f"sample_format must be one of {sorted(XM_SAMPLE_FORMATS)}, got {sample_format!r}"
        )

    pcm_bank = []
    duty_fallback = (12, 25, 50, 75)
    for i in range(15):
        xm_idx = i + 1
        if i < len(duty_inst):
            di = duty_inst[i]
            raw, loop = pulse_from_uge(
                int(di.get("duty_raw", 2)),
                int(di.get("init_vol", 15)),
                fw_chip,
                compact_pcm=True,
            )
            pcm_bank.append(
                xm_sample_pack_chip_pcm(raw, 0, loop, sample_format=sf)
            )
            if verbose:
                nm = di.get("name", "")[:40]
                print(
                    f"[uge2xm] ins {xm_idx:02d} pulse D{i + 1:02d} name={nm!r} "
                    f"duty_raw={di.get('duty_raw')} init_vol={di.get('init_vol')} "
                    f"ref_note_huge={XM_SAMPLE_REF_HUGE_NOTE} pcm_samples={len(raw)} "
                    f"loop_period={loop} sample_format={sf}",
                    flush=True,
                )
        else:
            raw = duty_pulse_pcm(duty_fallback[i % 4])
            raw = raw[:PULSE_PERIOD]
            pcm_bank.append(
                xm_sample_pack_chip_pcm(raw, 0, PULSE_PERIOD, sample_format=sf)
            )
            if verbose:
                print(
                    f"[uge2xm] ins {xm_idx:02d} pulse D{i + 1:02d} (placeholder) "
                    f"pcm_samples={len(raw)} loop_period={PULSE_PERIOD} sample_format={sf}",
                    flush=True,
                )
    for i in range(15):
        xm_idx = 15 + i + 1
        if i < len(wave_inst) and wavetables:
            wi = wave_inst[i]
            idx = int(wi.get("wavetable_index", 0))
            idx = max(0, min(len(wavetables) - 1, idx))
            raw_ol = int(wi.get("output_level", 1))
            ol = (raw_ol >> 5) & 3 if raw_ol > 3 else raw_ol & 3
            # NR32 0 = mute in hardware; still export a full-level sample so XM isn't silent noise.
            if ol == 0:
                ol = 1
            raw, loop = wave_pcm_from_uge(
                wavetables[idx], ol, fw_chip, compact_pcm=True
            )
            pcm_bank.append(
                xm_sample_pack_chip_pcm(raw, 0, loop, sample_format=sf)
            )
            if verbose:
                nm = wi.get("name", "")[:40]
                print(
                    f"[uge2xm] ins {xm_idx:02d} wave W{i + 1:02d} name={nm!r} "
                    f"wavetable_index={idx} output_level={ol} "
                    f"ref_note_huge={XM_SAMPLE_REF_HUGE_NOTE} pcm_samples={len(raw)} "
                    f"loop_period={loop} sample_format={sf}",
                    flush=True,
                )
        else:
            raw = wave_triangle_pcm()
            raw = raw[:WAVE_PERIOD]
            pcm_bank.append(
                xm_sample_pack_chip_pcm(raw, 0, WAVE_PERIOD, sample_format=sf)
            )
            if verbose:
                print(
                    f"[uge2xm] ins {xm_idx:02d} wave W{i + 1:02d} (placeholder) "
                    f"pcm_samples={len(raw)} loop_period={WAVE_PERIOD} sample_format={sf}",
                    flush=True,
                )
    instr_relative_notes: list[int] = [XM_SAMPLE_RELATIVE_NOTE] * NUM_META_INSTR
    noise_xm_meta: list[Optional[bytes]] = [None] * 30  # pulse+wave untouched
    for i in range(15):
        xm_idx = 30 + i + 1
        if i < len(noise_inst):
            ni = noise_inst[i]
            note_c = int(noise_note_pref.get(i + 1, XM_SAMPLE_REF_HUGE_NOTE))
            instr_relative_notes[29 + i + 1] = xm_noise_relative_note(note_c)
            raw, l0, ll, xm_n, noise_pcm_baked = noise_pcm_huge_xm(
                ni,
                note_c,
                xm_bpm=float(xm_bpm),
                compact_pcm=True,
                bake_noise_volume_envelope=bake_vol,
            )
            if (
                bake_noise_envelope == "normalized"
                and noise_pcm_baked
            ):
                raw = normalize_chip_pcm_u8_peak(raw)
            ext_n: Optional[bytes] = None
            if xm_n and isinstance(xm_n, Mapping):
                ext_n = pack_xm_instrument_ext208(
                    vol_points=list(xm_n["vol_points"]),
                    vol_sus=int(xm_n.get("vol_sus", 0)),
                    vol_loop_start=int(xm_n.get("vol_loop_start", 0)),
                    vol_loop_end=int(xm_n.get("vol_loop_end", 0)),
                    vol_type=int(xm_n.get("vol_type", 1)),
                    fadeout=int(xm_n.get("fadeout", 256)),
                )
            noise_xm_meta.append(ext_n)
            pcm_bank.append(
                xm_sample_pack_chip_pcm(raw, l0, ll, sample_format=sf)
            )
            if verbose:
                vd = int(ni.get("vol_dir", ST_UP)) & 0xFFFFFFFF
                vdir_s = "up" if vd == ST_UP else "down"
                nm = ni.get("name", "")[:40]
                xtra = ""
                if xm_n:
                    pts = xm_n.get("vol_points", ())
                    xtra = f" xm_vol_env={pts!s} fadeout={xm_n.get('fadeout')}"
                print(
                    f"[uge2xm] ins {xm_idx:02d} noise N{i + 1:02d} name={nm!r} "
                    f"init_vol={ni.get('init_vol')} vol_spd={ni.get('vol_spd')} "
                    f"dir={vdir_s} len_en={ni.get('len_enabled')} gb_len={ni.get('gb_len')} "
                    f"dividing_ratio={ni.get('dividing_ratio')} counter_step={ni.get('counter_step')} "
                    f"note_c_domCH4={note_c} xi_rel_note={instr_relative_notes[30 + i]} "
                    f"pcm_samples={len(raw)} xi_loop_start={l0} xi_loop_len={ll} "
                    f"sample_format={sf}"
                    f"{xtra}",
                    flush=True,
                )
        else:
            noise_xm_meta.append(None)
            raw = noise_pcm(0xACE1 + i)
            pcm_bank.append(
                xm_sample_pack_chip_pcm(raw, 0, 0, sample_format=sf)
            )
            if verbose:
                print(
                    f"[uge2xm] ins {xm_idx:02d} noise N{i + 1:02d} (placeholder) "
                    f"pcm_samples={len(raw)} xi_loop=(0,0) sample_format={sf}",
                    flush=True,
                )

    if len(noise_xm_meta) != NUM_META_INSTR:
        raise RuntimeError("noise_xm_meta length mismatch")

    xm_instr_remap: Optional[dict[int, int]] = None
    num_meta = NUM_META_INSTR
    if remove_unused:
        used = collect_used_xm_instrument_indices(patterns, orders, order_len)
        if used:
            sorted_used = sorted(used)
            xm_instr_remap = {old: new + 1 for new, old in enumerate(sorted_used)}
            pcm_bank = [pcm_bank[old - 1] for old in sorted_used]
            instr_names = [instr_names[old - 1] for old in sorted_used]
            instr_relative_notes = [instr_relative_notes[old - 1] for old in sorted_used]
            noise_xm_meta = [noise_xm_meta[old - 1] for old in sorted_used]
            num_meta = len(sorted_used)
            if verbose:
                dropped = NUM_META_INSTR - num_meta
                print(
                    f"[uge2xm] --remove-unused: {num_meta} instruments kept, "
                    f"{dropped} dropped (unused in patterns)",
                    flush=True,
                )
        elif verbose:
            print(
                "[uge2xm] --remove-unused: no instrument column references in patterns; "
                "keeping all 45 instruments",
                flush=True,
            )

    with open(xm_path, "wb") as out:
        write_xm_module_header(
            out,
            title,
            order_len,
            order_len,
            num_meta,
            xm_speed,
            xm_bpm,
        )
        for pos in range(order_len):
            pdata = build_pattern_data(
                patterns,
                orders,
                pos,
                ticks_per_row,
                xm_speed,
                c_envelope_volslide=c_envelope_volslide,
                timer_hz=timer_hz,
                xm_instr_remap=xm_instr_remap,
            )
            write_pattern(out, pdata)
        for i in range(num_meta):
            delta, l0, ll, sbits = pcm_bank[i]
            write_instrument(
                out,
                instr_names[i],
                delta,
                l0,
                ll,
                relative_note=instr_relative_notes[i],
                ext208=noise_xm_meta[i],
                sample_volume=xm_svol,
                sample_bits=sbits,
            )

    print(
        f"wrote {xm_path}: {order_len} patterns, {num_meta} instruments; "
        f"XM speed={xm_speed} bpm={xm_bpm}; sample_vol={xm_svol}/64 (FT2 0..64); "
        f"noise_env="
        f"{'baked(' + bake_noise_envelope + ')' if bake_noise_envelope else 'xi'}; "
        f"sample_format={sf}; "
        f"C-to-A volslide={'on' if c_envelope_volslide else 'off'}; "
        f"strip_unused={'on' if remove_unused else 'off'}; "
        f"(hUGE ticks/row={ticks_per_row}, timer_hz≈{timer_hz:.4f}, "
        f"hUGE ~{huge_beat_bpm} BPM if 4 rows/beat"
        + ("" if not d.get("timer_enabled") else f", timer divider={d.get('timer_divider')}")
        + ")"
    )


def main():
    ap = argparse.ArgumentParser(description="Convert .uge to .xm")
    ap.add_argument("input", help="input .uge")
    ap.add_argument("output", nargs="?", help="output .xm (default: input basename)")
    ap.add_argument(
        "--bpm",
        type=int,
        default=None,
        help="override XM BPM (default: derive from hUGE ticks/row and timer like tracker.pas)",
    )
    ap.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="log one line per XM instrument (45) with UGE fields and sample sizes",
    )
    ap.add_argument(
        "--c-envelope-volslide",
        action="store_true",
        help="approximate hUGE C** envelope down/up with a multi-row linear ramp in the XM "
        "volume column (duration ~ DMG steps*(P+1)/64 s); not a single-tick Axy nibble",
    )
    ap.add_argument(
        "--sample-format",
        choices=("u8", "s8", "u16", "s16"),
        default="s8",
        metavar="FMT",
        help="embedded XM sample encoding for all instruments: u8=8-bit delta on chip bytes "
        "(OpenMPT unsigned view); s8=8-bit signed wire + delta (OpenMPT default); "
        "s16=16-bit (XM 0x10); u16=same wire as s16 (alias). Default: s8",
    )
    ap.add_argument(
        "--bake-noise-envelope",
        nargs="?",
        const="raw",
        default=None,
        choices=("raw", "normalized"),
        metavar="MODE",
        help="decaying CH4: bake volume into PCM (skip XI envelope where applicable). "
        "raw=same as before; normalized=peak-normalize that baked PCM. "
        "Omit the flag to keep XI volume envelope. "
        "`--bake-noise-envelope` alone is the same as raw.",
    )
    ap.add_argument(
        "--remove-unused",
        action="store_true",
        help="omit XM instruments (and embedded samples) never referenced on a note in any pattern "
        "order; renumber pattern instrument column to 1..N",
    )
    args = ap.parse_args()
    outp = args.output
    if not outp:
        base, _ = os.path.splitext(args.input)
        outp = base + ".xm"
    uge_to_xm(
        args.input,
        outp,
        bpm_override=args.bpm,
        verbose=args.verbose,
        c_envelope_volslide=args.c_envelope_volslide,
        bake_noise_envelope=args.bake_noise_envelope,
        remove_unused=args.remove_unused,
        sample_format=args.sample_format,
    )


if __name__ == "__main__":
    main()
