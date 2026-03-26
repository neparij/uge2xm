# Copyright (c) 2026 Nikolai Laptev (neparij)
# SPDX-License-Identifier: Zlib

"""Placeholder sample generators for uge2xm (45 Game Boy style timbres)."""

from .xm_placeholders import (
    duty_pulse_pcm,
    wave_triangle_pcm,
    noise_pcm,
    chip_u8_to_xm_wire8,
    xm_wire8_to_chip_u8,
    encode_delta_u8,
    encode_chip_pcm_delta_xm,
    XM_SAMPLE_FORMATS,
    xm_sample_pack_chip_pcm,
    encode_delta_u16_le,
    decode_delta_u8,
    decode_delta_u8_to_chip_u8,
    decode_delta_u16_le,
    pcm_u8_chip_to_xm_s16le_body,
    PULSE_PERIOD,
    WAVE_PERIOD,
    NOISE_LENGTH,
)

__all__ = [
    "duty_pulse_pcm",
    "wave_triangle_pcm",
    "noise_pcm",
    "chip_u8_to_xm_wire8",
    "xm_wire8_to_chip_u8",
    "encode_delta_u8",
    "encode_chip_pcm_delta_xm",
    "XM_SAMPLE_FORMATS",
    "xm_sample_pack_chip_pcm",
    "encode_delta_u16_le",
    "decode_delta_u8",
    "decode_delta_u8_to_chip_u8",
    "decode_delta_u16_le",
    "pcm_u8_chip_to_xm_s16le_body",
    "PULSE_PERIOD",
    "WAVE_PERIOD",
    "NOISE_LENGTH",
]
