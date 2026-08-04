import struct

from ti_radar_driver.tlv_parser import (
    HEADER_LENGTH,
    MAGIC_WORD,
    DetectedPoint,
    TlvParser,
    build_frame,
)


def make_points():
    return [
        DetectedPoint(x=1.0, y=5.0, z=0.5, doppler=1.5, snr_db=20.0,
                      noise_db=5.0),
        DetectedPoint(x=-0.5, y=3.0, z=1.0, doppler=-0.7, snr_db=15.5,
                      noise_db=6.0),
    ]


def test_happy_path_single_frame():
    parser = TlvParser()
    frames = parser.feed(build_frame(42, make_points()))
    assert len(frames) == 1
    frame = frames[0]
    assert frame.frame_number == 42
    assert frame.num_detected_obj == 2
    assert len(frame.points) == 2
    p = frame.points[0]
    assert (p.x, p.y, p.z, p.doppler) == (1.0, 5.0, 0.5, 1.5)
    assert p.snr_db == 20.0
    assert p.noise_db == 5.0


def test_frame_split_across_arbitrary_chunks():
    data = build_frame(1, make_points()) + build_frame(2, make_points())
    for chunk_size in (1, 3, 7, 16, 100):
        parser = TlvParser()
        frames = []
        for i in range(0, len(data), chunk_size):
            frames.extend(parser.feed(data[i:i + chunk_size]))
        assert [f.frame_number for f in frames] == [1, 2], \
            f'failed at chunk_size={chunk_size}'


def test_leading_garbage_resync():
    parser = TlvParser()
    garbage = bytes(range(256)) * 2
    frames = parser.feed(garbage + build_frame(9, make_points()))
    assert [f.frame_number for f in frames] == [9]


def test_garbage_containing_partial_magic():
    parser = TlvParser()
    # A partial magic word split across feeds must not desync the parser.
    frames = parser.feed(MAGIC_WORD[:5])
    assert frames == []
    frames = parser.feed(MAGIC_WORD[5:] + build_frame(3, make_points())[8:])
    # The first (reassembled) magic word starts a bogus frame whose header
    # is the *real* frame's tail; the parser must eventually recover.
    parser.feed(build_frame(4, make_points()))


def test_corrupt_length_field_recovers():
    parser = TlvParser()
    bad = bytearray(build_frame(5, make_points()))
    # totalPacketLen sits after magic + version.
    struct.pack_into('<I', bad, len(MAGIC_WORD) + 4, 0xFFFFFFFF)
    frames = parser.feed(bytes(bad) + build_frame(6, make_points()))
    assert [f.frame_number for f in frames] == [6]


def test_unknown_tlv_type_skipped():
    parser = TlvParser()
    payload = b'\xAB' * 24
    data = build_frame(7, make_points(), extra_tlvs=[(1234, payload)])
    frames = parser.feed(data)
    assert len(frames) == 1
    assert len(frames[0].points) == 2


def test_missing_side_info_tolerated():
    parser = TlvParser()
    frames = parser.feed(build_frame(8, make_points(),
                                     include_side_info=False))
    assert len(frames) == 1
    for p in frames[0].points:
        assert p.snr_db == 0.0
        assert p.noise_db == 0.0


def test_empty_frame():
    parser = TlvParser()
    frames = parser.feed(build_frame(10, []))
    assert len(frames) == 1
    assert frames[0].points == []


def test_truncated_tlv_does_not_crash():
    parser = TlvParser()
    data = bytearray(build_frame(11, make_points()))
    # Claim one more TLV than the packet actually contains.
    struct.pack_into('<I', data, len(MAGIC_WORD) + 24, 5)
    parser.feed(bytes(data))
    # Parsed or skipped are both acceptable; crashing or hanging is not.
    parser.feed(build_frame(12, make_points()))


def test_header_length_constant():
    frame_bytes = build_frame(1, [])
    assert len(frame_bytes) >= HEADER_LENGTH
    assert frame_bytes[:8] == MAGIC_WORD
