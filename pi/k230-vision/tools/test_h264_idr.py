"""h264_idr.IdrExtractor（离线）：
    cd pi/k230-vision && python -m pytest tools/test_h264_idr.py -q
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "pi"))

from h264_idr import IdrExtractor  # noqa: E402

SPS = b"\x67\x42\x00\x1f\xaa"
PPS = b"\x68\xce\x3c\x80"
IDR1 = b"\x65\x88\x80\x10\x01"
IDR2 = b"\x65\x00\x40\x20\x02"      # 同一张 IDR 的第二个片
P = b"\x41\x9a\x01\x02"


def _au(*nals, four=True):
    sc = b"\x00\x00\x00\x01" if four else b"\x00\x00\x01"
    return b"".join(sc + n for n in nals)


def _expect(*nals):
    return b"".join(b"\x00\x00\x00\x01" + n for n in nals)


def test_extracts_sps_pps_idr_and_skips_p():
    s = _au(SPS, PPS, IDR1, IDR2) + _au(P) + _au(P) + _au(SPS)
    ex = IdrExtractor()
    out = ex.feed(s)
    assert out == [_expect(SPS, PPS, IDR1, IDR2)]


def test_byte_by_byte_feed_matches():
    s = _au(SPS, PPS, IDR1) + _au(P, four=False) + _au(SPS, PPS, IDR2) + _au(P) + _au(P)
    ex = IdrExtractor()
    got = []
    for i in range(len(s)):
        got += ex.feed(s[i:i + 1])
    assert got == [_expect(SPS, PPS, IDR1), _expect(SPS, PPS, IDR2)]


def test_idr_before_parameter_sets_is_dropped():
    ex = IdrExtractor()
    assert ex.feed(_au(IDR1) + _au(P) + _au(P)) == []
