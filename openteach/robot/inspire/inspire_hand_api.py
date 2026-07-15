"""
Low-level driver for the Inspire-Robots RH56 dexterous hand (6 DOF), over
RS485 using the vendor's proprietary 0xEB 0x90 protocol.

Reference: "THE DEXTEROUS HAND RH56 SERIES USER MANUAL" (Inspire-Robots),
sections 2.2 (RS485 register read/write) and 2.4 (register map).

DOF order (6 actuators), matches the manual's ANGLE_SET/ANGLE_ACT register
groups exactly:
    0: little finger      1: ring finger     2: middle finger
    3: index finger       4: thumb bending   5: thumb rotation

Angle convention (ANGLE_SET / ANGLE_ACT, range 0-1000):
    1000 = fully OPEN (finger extended)
       0 = fully CLOSED (finger bent toward palm)
      -1 = "no action" (leave that DOF unchanged)  [ANGLE_SET only]

Serial defaults (manual 2.2): 115200 baud, 8 data bits, 1 stop bit, no parity.
"""

import struct
import time

try:
    import serial  # pyserial
except ImportError as e:  # pragma: no cover
    serial = None
    _serial_import_error = e

# Register start addresses (manual section 2.4)
ADDR_ANGLE_SET = 0x05CE   # 1486, 6 shorts (W/R) - commanded angle per DOF
ADDR_ANGLE_ACT = 0x060A   # 1546, 6 shorts (R)   - actual angle per DOF
ADDR_SPEED_SET = 0x05F2   # 1522, 6 shorts (W/R) - speed per DOF (0-1000)
ADDR_FORCE_SET = 0x05DA   # 1498, 6 shorts (W/R) - force threshold per DOF (g)
ADDR_POS_ACT   = 0x0534   # 1534, 6 shorts (R)   - actual actuator position
ADDR_CLEAR_ERR = 0x03EC   # 1004, 1 byte (W)     - CLEAR_ERROR

WRITE_FLAG = 0x12
READ_FLAG = 0x11

NUM_DOF = 6
DOF_NAMES = ['little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rot']


class InspireHand:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, hand_id=1,
                 timeout=0.2, dry_run=False):
        """dry_run=True skips opening the serial port and never transmits -- use
        it to test framing/retargeting without the hardware connected."""
        self.hand_id = hand_id
        self.dry_run = dry_run
        self._ser = None
        if not dry_run:
            if serial is None:
                raise ImportError(
                    'pyserial is required to talk to the Inspire hand: '
                    'pip install pyserial (original error: %s)' % _serial_import_error)
            self._ser = serial.Serial(
                port=port, baudrate=baudrate, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=timeout)

    # ---- frame construction ------------------------------------------------
    @staticmethod
    def _checksum(body):
        # Manual 2.2.1: checksum = low byte of the sum of every byte AFTER the
        # 2-byte 0xEB 0x90 header, up to (not including) the checksum itself.
        return sum(body) & 0xFF

    def _build_write_frame(self, addr, data_bytes):
        reg_len = len(data_bytes)
        body = [
            self.hand_id,
            reg_len + 3,          # frame data length
            WRITE_FLAG,
            addr & 0xFF,          # address low
            (addr >> 8) & 0xFF,   # address high
            *data_bytes,
        ]
        frame = [0xEB, 0x90] + body + [self._checksum(body)]
        return bytes(frame)

    def _build_read_frame(self, addr, reg_len):
        body = [
            self.hand_id,
            0x04,
            READ_FLAG,
            addr & 0xFF,
            (addr >> 8) & 0xFF,
            reg_len,
        ]
        frame = [0xEB, 0x90] + body + [self._checksum(body)]
        return bytes(frame)

    @staticmethod
    def _shorts_to_le_bytes(values):
        out = []
        for v in values:
            v = int(v) & 0xFFFF        # two's complement for -1 -> 0xFFFF
            out += [v & 0xFF, (v >> 8) & 0xFF]   # little-endian, low byte first
        return out

    # ---- register access ---------------------------------------------------
    def _write_register(self, addr, data_bytes):
        frame = self._build_write_frame(addr, data_bytes)
        if self.dry_run:
            return frame
        self._ser.write(frame)
        # read/discard the short acknowledgement frame if present
        self._ser.read(9)
        return frame

    def _read_register(self, addr, reg_len):
        if self.dry_run:
            return [0] * (reg_len // 2)
        self._ser.reset_input_buffer()
        self._ser.write(self._build_read_frame(addr, reg_len))
        resp = self._ser.read(7 + reg_len + 1)  # header(2)+id+len+flag+addr(2)+data+chk
        if len(resp) < 7 + reg_len + 1:
            return None
        data = resp[7:7 + reg_len]
        return list(struct.unpack('<%dh' % (reg_len // 2), data))

    # ---- public API --------------------------------------------------------
    def set_angles(self, angles):
        """Command all 6 DOF. angles: iterable of 6 ints in [0,1000] (or -1 to
        leave a DOF unchanged). 1000=open, 0=closed."""
        assert len(angles) == NUM_DOF, 'expected 6 angle values'
        clamped = []
        for a in angles:
            a = int(round(a))
            if a != -1:
                a = max(0, min(1000, a))
            clamped.append(a)
        return self._write_register(ADDR_ANGLE_SET, self._shorts_to_le_bytes(clamped))

    def get_angles(self):
        """Read actual angle of all 6 DOF (0-1000), or None on read failure."""
        return self._read_register(ADDR_ANGLE_ACT, 12)

    def set_speed(self, speeds):
        assert len(speeds) == NUM_DOF
        return self._write_register(ADDR_SPEED_SET, self._shorts_to_le_bytes(speeds))

    def set_force(self, forces):
        assert len(forces) == NUM_DOF
        return self._write_register(ADDR_FORCE_SET, self._shorts_to_le_bytes(forces))

    def clear_error(self):
        return self._write_register(ADDR_CLEAR_ERR, [1])

    def open_hand(self):
        self.set_angles([1000] * NUM_DOF)

    def close_hand(self):
        # thumb rotation left open (1000) so the thumb doesn't fight the fingers
        self.set_angles([0, 0, 0, 0, 0, 1000])

    def close(self):
        if self._ser is not None:
            self._ser.close()
