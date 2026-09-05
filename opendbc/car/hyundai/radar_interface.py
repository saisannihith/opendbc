import math
import time
from dataclasses import dataclass

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import CAR, DBC

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from openpilot.common.swaglog import cloudlog

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

CARNIVAL_4TH_GEN_OBJECT_START_ADDR = 0x180
CARNIVAL_4TH_GEN_OBJECT_END_ADDR = 0x184
CARNIVAL_4TH_GEN_OBJECT_BUS = 1
CARNIVAL_4TH_GEN_OBJECT_LEN = 32
CARNIVAL_4TH_GEN_OBJECT_LOG_INTERVAL = 1.0
CARNIVAL_4TH_GEN_TRACK_ID_BASE = 0xC4100
CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE = 0.25
CARNIVAL_4TH_GEN_CONFIRMATION_MIN_PERSIST = 3
CARNIVAL_4TH_GEN_CONFIRMATION_MAX_GAP = 0.15
CARNIVAL_4TH_GEN_CONFIRMATION_VELOCITY_SPEC = (91, 11, 0.05, 2.4)
CARNIVAL_4TH_GEN_CONFIRMATION_MAX_ABS_Y = 50.0
CARNIVAL_4TH_GEN_CONFIRMATION_MAX_ABS_V = 60.0
CARNIVAL_4TH_GEN_FRAME_CHECKSUM_XOR = 0x9F5B


def get_little_unsigned(dat: bytes, start: int, size: int) -> int:
  return (int.from_bytes(dat, "little", signed=False) >> start) & ((1 << size) - 1)


def get_little_signed(dat: bytes, start: int, size: int) -> int:
  val = get_little_unsigned(dat, start, size)
  return val - (1 << size) if val & (1 << (size - 1)) else val


def carnival_radar_frame_checksum(address: int, dat: bytes) -> int:
  crc = 0
  for value in dat[2:]:
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ value]) & 0xFFFF
  for value in (address & 0xFF, (address >> 8) & 0xFF):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ value]) & 0xFFFF
  return crc ^ CARNIVAL_4TH_GEN_FRAME_CHECKSUM_XOR


def carnival_radar_frame_valid(address: int, dat: bytes) -> bool:
  return (len(dat) == CARNIVAL_4TH_GEN_OBJECT_LEN and
          int.from_bytes(dat[:2], "little") == carnival_radar_frame_checksum(address, dat))


def decode_carnival_confirmation_velocity(dat: bytes, bit_offset: int) -> float:
  start, size, scale, offset = CARNIVAL_4TH_GEN_CONFIRMATION_VELOCITY_SPEC
  return get_little_signed(dat, bit_offset + start, size) * scale + offset


@dataclass(frozen=True)
class CarnivalRadarObject:
  raw_track_id: int
  heartbeat: int
  valid_count: int
  state_alt: int
  state: int
  metadata_50_63: int
  d_rel: float
  y_rel: float
  v_rel: float


def decode_carnival_radar_object(dat: bytes, bit_offset: int) -> CarnivalRadarObject:
  return CarnivalRadarObject(
    raw_track_id=get_little_unsigned(dat, bit_offset + 42, 8),
    heartbeat=get_little_unsigned(dat, bit_offset + 124, 4),
    valid_count=get_little_unsigned(dat, bit_offset + 32, 8),
    state_alt=get_little_unsigned(dat, bit_offset + 51, 4),
    state=get_little_unsigned(dat, bit_offset + 55, 3),
    metadata_50_63=get_little_unsigned(dat, bit_offset + 50, 14),
    d_rel=get_little_unsigned(dat, bit_offset + 64, 13) * 0.05,
    y_rel=get_little_signed(dat, bit_offset + 78, 11) * 0.05,
    v_rel=decode_carnival_confirmation_velocity(dat, bit_offset),
  )


def carnival_radar_object_valid(obj: CarnivalRadarObject) -> bool:
  return (obj.valid_count != 0 and obj.raw_track_id != 0 and
          0.5 <= obj.d_rel <= 220.0 and
          abs(obj.y_rel) <= CARNIVAL_4TH_GEN_CONFIRMATION_MAX_ABS_Y and
          abs(obj.v_rel) <= CARNIVAL_4TH_GEN_CONFIRMATION_MAX_ABS_V)


def carnival_confirmation_continuous(prev: tuple[float, float, float, float] | None, now: float,
                                     obj: CarnivalRadarObject) -> bool:
  if prev is None:
    return False
  prev_t, prev_d, prev_y, prev_v = prev
  dt = now - prev_t
  return (0.0 <= dt <= CARNIVAL_4TH_GEN_CONFIRMATION_MAX_GAP and
          abs(obj.d_rel - prev_d) <= max(1.5, 60.0 * max(dt, 0.0)) and
          abs(obj.y_rel - prev_y) <= max(1.0, 20.0 * max(dt, 0.0)) and
          abs(obj.v_rel - prev_v) <= 8.0)


# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1

    self.radar_off_can = CP.radarUnavailable
    self.carnival_object_probe = CP.carFingerprint == CAR.KIA_CARNIVAL_4TH_GEN
    self.carnival_object_probe_prev: dict[tuple[int, int], tuple[float, float]] = {}
    self.carnival_object_probe_last_log = 0.0
    self.carnival_object_probe_seen = 0
    self.carnival_object_probe_valid = 0
    self.carnival_object_probe_crc_invalid = 0
    self.carnival_confirmation_tracks: dict[int, tuple[float, float, float, float]] = {}
    self.carnival_confirmation_prev: dict[int, tuple[float, float, float, float]] = {}
    self.carnival_confirmation_persist: dict[int, int] = {}
    self.rcp = get_radar_can_parser(CP)

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.carnival_object_probe and can_strings is not None:
      self._update_carnival_object_probe(can_strings)

    if self.radar_off_can or (self.rcp is None):
      rr = super().update(None)
      if rr is not None and self.carnival_object_probe:
        self._add_carnival_confirmation_tracks(rr)
      return rr

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    if self.carnival_object_probe:
      self._add_carnival_confirmation_tracks(rr)

    return rr

  def _update_carnival_object_probe(self, can_strings):
    now = time.monotonic()
    sample: CarnivalRadarObject | None = None
    batch_objects: dict[int, CarnivalRadarObject] = {}
    conflicting_ids: set[int] = set()

    for _, frames in can_strings:
      for address, dat, src in frames:
        if src != CARNIVAL_4TH_GEN_OBJECT_BUS:
          continue
        if not (CARNIVAL_4TH_GEN_OBJECT_START_ADDR <= address <= CARNIVAL_4TH_GEN_OBJECT_END_ADDR):
          continue
        if len(dat) != CARNIVAL_4TH_GEN_OBJECT_LEN:
          continue

        self.carnival_object_probe_seen += 1
        if not carnival_radar_frame_valid(address, dat):
          self.carnival_object_probe_crc_invalid += 1
          continue
        for bit_offset in (0, 128):
          obj = decode_carnival_radar_object(dat, bit_offset)
          if not carnival_radar_object_valid(obj):
            continue
          self.carnival_object_probe_valid += 1
          if address == CARNIVAL_4TH_GEN_OBJECT_START_ADDR and bit_offset == 0:
            sample = obj
          previous_in_batch = batch_objects.get(obj.raw_track_id)
          if previous_in_batch is not None and previous_in_batch != obj:
            conflicting_ids.add(obj.raw_track_id)
          else:
            batch_objects[obj.raw_track_id] = obj

    for raw_track_id in conflicting_ids:
      batch_objects.pop(raw_track_id, None)
      self.carnival_confirmation_tracks.pop(raw_track_id, None)
      self.carnival_confirmation_prev.pop(raw_track_id, None)
      self.carnival_confirmation_persist.pop(raw_track_id, None)

    for raw_track_id, obj in batch_objects.items():
      previous = self.carnival_confirmation_prev.get(raw_track_id)
      continuous = carnival_confirmation_continuous(previous, now, obj)
      persist = self.carnival_confirmation_persist.get(raw_track_id, 0) + 1 if continuous else 1
      if not continuous:
        self.carnival_confirmation_tracks.pop(raw_track_id, None)
      self.carnival_confirmation_prev[raw_track_id] = (now, obj.d_rel, obj.y_rel, obj.v_rel)
      self.carnival_confirmation_persist[raw_track_id] = persist
      if persist >= CARNIVAL_4TH_GEN_CONFIRMATION_MIN_PERSIST:
        self.carnival_confirmation_tracks[raw_track_id] = (now, obj.d_rel, obj.y_rel, obj.v_rel)

    self._expire_carnival_confirmation_tracks(now)
    if sample is None or now - self.carnival_object_probe_last_log < CARNIVAL_4TH_GEN_OBJECT_LOG_INTERVAL:
      return

    sample_key = (CARNIVAL_4TH_GEN_OBJECT_START_ADDR, 1)
    sample_prev = self.carnival_object_probe_prev.get(sample_key)
    d_dot = float("nan")
    if sample_prev is not None:
      prev_t, prev_d = sample_prev
      dt = now - prev_t
      if 0.01 <= dt <= 1.0:
        d_dot = (sample.d_rel - prev_d) / dt
    self.carnival_object_probe_prev[sample_key] = (now, sample.d_rel)
    d_dot_str = "nan" if not math.isfinite(d_dot) else f"{d_dot:.2f}"
    cloudlog.warning("".join((
      "Carnival 4th gen radar probe: ",
      f"addr=0x{CARNIVAL_4TH_GEN_OBJECT_START_ADDR:x} slot=1 rawTrackId={sample.raw_track_id} ",
      f"validCount={sample.valid_count} heartbeat={sample.heartbeat} ",
      f"dRel={sample.d_rel:.2f} yRel={sample.y_rel:.2f} vRel={sample.v_rel:.2f} ",
      f"dDot={d_dot_str} shadowDistance=YES ",
      f"r0100Track={sample.raw_track_id in self.carnival_confirmation_tracks} ",
      f"publishedTracks={len(self.carnival_confirmation_tracks)} publishReady={bool(self.carnival_confirmation_tracks)} ",
      f"kinematicsDecoded=YES controlReady=MODEL_FUSED_TARGET_QUALIFIED conflicts={len(conflicting_ids)} ",
      f"seen={self.carnival_object_probe_seen} ",
      f"valid={self.carnival_object_probe_valid}",
      f" crcInvalid={self.carnival_object_probe_crc_invalid}",
    )))
    self.carnival_object_probe_last_log = now

  def _expire_carnival_confirmation_tracks(self, now):
    stale = [raw_track_id for raw_track_id, (track_time, *_) in self.carnival_confirmation_tracks.items()
             if now - track_time > CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE]
    for raw_track_id in stale:
      self.carnival_confirmation_tracks.pop(raw_track_id, None)
      self.carnival_confirmation_prev.pop(raw_track_id, None)
      self.carnival_confirmation_persist.pop(raw_track_id, None)

  def _add_carnival_confirmation_tracks(self, rr):
    self._expire_carnival_confirmation_tracks(time.monotonic())
    if not self.carnival_confirmation_tracks:
      return

    points = list(rr.points)
    for raw_track_id, (_, d_rel, y_rel, v_rel) in sorted(self.carnival_confirmation_tracks.items()):
      pt = structs.RadarData.RadarPoint()
      pt.trackId = CARNIVAL_4TH_GEN_TRACK_ID_BASE + raw_track_id
      pt.measured = True
      pt.dRel = float(d_rel)
      pt.yRel = float(y_rel)
      pt.vRel = float(v_rel)
      pt.aRel = float("nan")
      pt.yvRel = float("nan")
      points.append(pt)
    rr.points = points

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.use_radar_interface_ext:
      return self.update_ext(ret)

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret
