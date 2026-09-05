from opendbc.car.hyundai.radar_interface import (
  CARNIVAL_4TH_GEN_OBJECT_START_ADDR,
  carnival_confirmation_continuous,
  carnival_radar_frame_checksum,
  carnival_radar_frame_valid,
  carnival_radar_object_valid,
  decode_carnival_radar_object,
)


def _set_bits(dat: bytearray, start: int, size: int, value: int) -> None:
  raw = int.from_bytes(dat, "little")
  mask = ((1 << size) - 1) << start
  raw = (raw & ~mask) | ((value & ((1 << size) - 1)) << start)
  dat[:] = raw.to_bytes(len(dat), "little")


def _set_signed(dat: bytearray, start: int, size: int, value: int) -> None:
  if value < 0:
    value += 1 << size
  _set_bits(dat, start, size, value)


def _sample_frame() -> bytes:
  dat = bytearray(32)
  _set_bits(dat, 32, 8, 5)
  _set_bits(dat, 42, 8, 17)
  _set_bits(dat, 51, 4, 3)
  _set_bits(dat, 55, 3, 4)
  _set_bits(dat, 64, 13, 600)
  _set_signed(dat, 78, 11, -12)
  _set_signed(dat, 91, 11, -48)
  _set_bits(dat, 124, 4, 9)
  checksum = carnival_radar_frame_checksum(CARNIVAL_4TH_GEN_OBJECT_START_ADDR, bytes(dat))
  dat[0:2] = checksum.to_bytes(2, "little")
  return bytes(dat)


def test_carnival_r0100_crc_and_decode():
  dat = _sample_frame()
  assert carnival_radar_frame_valid(CARNIVAL_4TH_GEN_OBJECT_START_ADDR, dat)

  obj = decode_carnival_radar_object(dat, 0)
  assert obj.raw_track_id == 17
  assert obj.valid_count == 5
  assert obj.heartbeat == 9
  assert obj.d_rel == 30.0
  assert abs(obj.y_rel - -0.6) < 1e-9
  assert abs(obj.v_rel) < 1e-9
  assert carnival_radar_object_valid(obj)


def test_carnival_r0100_rejects_bad_crc():
  dat = bytearray(_sample_frame())
  dat[20] ^= 0x55
  assert not carnival_radar_frame_valid(CARNIVAL_4TH_GEN_OBJECT_START_ADDR, bytes(dat))


def test_carnival_r0100_continuity_gate():
  obj = decode_carnival_radar_object(_sample_frame(), 0)
  prev = (10.0, obj.d_rel - 0.2, obj.y_rel + 0.1, obj.v_rel)
  assert carnival_confirmation_continuous(prev, 10.05, obj)
  assert not carnival_confirmation_continuous(prev, 10.30, obj)
