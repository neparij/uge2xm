# Copyright (c) 2026 Nikolai Laptev (neparij)
# SPDX-License-Identifier: Zlib

"""8-bit mono PCM placeholders for XM instruments (FT2/OpenMPT tuning)."""

import struct

# Pulse/wave: ~32 samples/period for FT2 C-4. Noise: one-shot burst (no XM loop), percussive.
PULSE_PERIOD = 32
PULSE_REPEAT = 64
WAVE_PERIOD = 32
WAVE_REPEAT = 64
NOISE_LENGTH = 320


def duty_pulse_pcm(duty_percent=50):
    """Band-limited-ish square: many periods, one period repeats in the loop."""
    lo, hi = 0x58, 0xA8
    n1 = max(1, int(PULSE_PERIOD * duty_percent / 100))
    n2 = PULSE_PERIOD - n1
    period = bytes([hi] * n1 + [lo] * n2)
    return period * PULSE_REPEAT


def wave_triangle_pcm():
    """Triangle wavetable; seamless period, smooth (less aliasing than harsh sine grid)."""
    n = WAVE_PERIOD
    out = bytearray(n)
    half = n // 2
    for i in range(half):
        out[i] = 0x48 + int((0xB0 - 0x48) * i / max(1, half - 1))
    for i in range(half, n):
        j = i - half
        out[i] = 0xB0 - int((0xB0 - 0x48) * j / max(1, n - half - 1))
    out[n - 1] = out[0]
    return bytes(out) * WAVE_REPEAT


def noise_pcm(seed=0xACE1):
    """Short decaying noise burst (one-shot); avoids sustained hiss in XM."""
    length = NOISE_LENGTH
    rng = seed & 0xFFFFFFFF
    raw = bytearray(length)
    for i in range(length):
        rng = (rng * 1664525 + 1013904223) & 0xFFFFFFFF
        raw[i] = max(0x40, min(0xC0, (rng & 0xFF)))
    exp = 1.35
    for i in range(length):
        t = i / max(1, length - 1)
        g = (1.0 - t) ** exp
        raw[i] = max(0, min(255, 0x80 + int((raw[i] - 0x80) * g)))
    sm = bytearray(length)
    for i in range(length):
        a = raw[i - 1] if i else raw[0]
        b = raw[i]
        c = raw[i + 1] if i + 1 < length else b
        sm[i] = (a + b + c) // 3
    return bytes(sm)


def chip_u8_to_xm_wire8(pcm_chip_u8: bytes) -> bytes:
    """Chip / hUGE style (128 = silence) to XM wire bytes for **signed int8** view (0 = silence).

    OpenMPT stores 8-bit samples as ``int8`` with **0** at DC; XM delta export uses successive
    **signed** sample values (see ``SampleIO::WriteSample``, deltaPCM). FT2/libxmp still accumulate
    deltas with ``acc = (acc + d) & 0xFF`` on those wire bytes; same formula as
    ``encode_delta_u8``, but absolute levels must use this mapping, not raw chip 0..255.
    """
    return bytes((int(b) - 128) & 0xFF for b in pcm_chip_u8)


def xm_wire8_to_chip_u8(wire: bytes) -> bytes:
    """Inverse of ``chip_u8_to_xm_wire8`` (after delta decode)."""
    return bytes((int(w) + 128) & 0xFF for w in wire)


def encode_delta_u8(wire_abs: bytes) -> bytes:
    """FT2 XM **8-bit** delta on **wire** bytes (0..255): ``d[i] = (w[i] - w[i-1]) & 0xFF``, ``w[-1]=0``.

    Decoded ``w`` is the unsigned byte form of **signed int8** audio (silence **0**, not 0x80).
    For chip PCM (128 = silence), run ``chip_u8_to_xm_wire8`` first, or use ``encode_chip_pcm_delta_xm``.
    """
    out = bytearray(len(wire_abs))
    prev = 0
    for i, x in enumerate(wire_abs):
        out[i] = (x - prev) & 0xFF
        prev = x
    return bytes(out)


def encode_chip_pcm_delta_xm(pcm_chip_u8: bytes) -> bytes:
    """Delta-encode chip-style PCM (0..255, 128 = silence) for OpenMPT-compatible XM 8-bit."""
    return encode_delta_u8(chip_u8_to_xm_wire8(pcm_chip_u8))


XM_SAMPLE_FORMATS = frozenset({"u8", "s8", "u16", "s16"})


def xm_sample_pack_chip_pcm(
    pcm_chip_u8: bytes,
    loop_start_frames: int = 0,
    loop_length_frames: int = 0,
    *,
    sample_format: str = "s8",
) -> tuple[bytes, int, int, int]:
    """Chip mono u8 (128 = silence) to XM delta + loop fields for ``write_instrument``.

    Returns ``(delta_bytes, loop_start_bytes, loop_length_bytes, sample_bits)``. Loop arguments are
    **frame** counts (one sample = one frame for 8-bit mono); for 16-bit they are multiplied by 2
    in the returned file offsets.

    - **u8**: FT2 delta on raw chip bytes (OpenMPT 8-bit \"unsigned\" / offset-binary view).
    - **s8**: map to signed wire (silence 0) then delta (OpenMPT 8-bit \"signed\" default).
    - **s16** / **u16**: 16-bit XM (type flag 0x10); same FT2 wire. ``u16`` is a supported alias.
    """
    sf = sample_format.strip().lower()
    if sf not in XM_SAMPLE_FORMATS:
        raise ValueError(
            f"sample_format must be one of {sorted(XM_SAMPLE_FORMATS)}, got {sample_format!r}"
        )
    ls = max(0, int(loop_start_frames))
    ll = max(0, int(loop_length_frames))
    if sf == "u8":
        return encode_delta_u8(pcm_chip_u8), ls, ll, 8
    if sf == "s8":
        return encode_chip_pcm_delta_xm(pcm_chip_u8), ls, ll, 8
    delta = encode_delta_u16_le(pcm_u8_chip_to_xm_s16le_body(pcm_chip_u8))
    return delta, ls * 2, ll * 2, 16


def pcm_u8_chip_to_xm_s16le_body(pcm_u8: bytes) -> bytes:
    """
    Map chip-style **s8 in offset binary** (128 = silence) to XM **16-bit signed** mono body (LE).

    Same linear map as a typical WAV export: ``s16 = clamp16((u8 - 128) * 256)``. OpenMPT/libxmp then
    delta-decode uint16 words with wrap; bit pattern matches signed int16 samples on the wire.
    """
    out = bytearray(len(pcm_u8) * 2)
    j = 0
    for b in pcm_u8:
        s = (int(b) - 128) * 256
        s = max(-32768, min(32767, s))
        struct.pack_into("<h", out, j, s)
        j += 2
    return bytes(out)


def encode_delta_u16_le(pcm16_le: bytes) -> bytes:
    """FT2 XM **16-bit** mono: each u16 LE word is a delta; decode ``acc = (acc + d) & 0xFFFF`` (libxmp/OpenMPT)."""
    if len(pcm16_le) % 2:
        raise ValueError("16-bit PCM body length must be even")
    out = bytearray(len(pcm16_le))
    prev = 0
    for i in range(0, len(pcm16_le), 2):
        val = struct.unpack_from("<H", pcm16_le, i)[0]
        d = (val - prev) & 0xFFFF
        struct.pack_into("<H", out, i, d)
        prev = val
    return bytes(out)


def decode_delta_u16_le(delta: bytes) -> bytes:
    """Inverse of ``encode_delta_u16_le`` (accumulated u16 LE absolute words as bytes)."""
    if len(delta) % 2:
        raise ValueError("16-bit delta length must be even")
    out = bytearray(len(delta))
    prev = 0
    for i in range(0, len(delta), 2):
        d = struct.unpack_from("<H", delta, i)[0]
        prev = (prev + d) & 0xFFFF
        struct.pack_into("<H", out, i, prev)
    return bytes(out)


def decode_delta_u8(delta: bytes) -> bytes:
    """Inverse of ``encode_delta_u8``: absolute **wire** bytes (signed int8 bit pattern; silence ``0``)."""
    out = bytearray(len(delta))
    prev = 0
    for i, d in enumerate(delta):
        prev = (prev + d) & 0xFF
        out[i] = prev
    return bytes(out)


def decode_delta_u8_to_chip_u8(delta: bytes) -> bytes:
    """Delta-decode then map wire bytes back to chip/hUGE PCM (128 = silence)."""
    return xm_wire8_to_chip_u8(decode_delta_u8(delta))
