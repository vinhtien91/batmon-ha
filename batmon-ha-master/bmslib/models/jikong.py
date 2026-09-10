"""
 JK, jikong

Manual
https://github.com/NEEY-electronic/JK/blob/JK-BMS/JKBMS%20INSTRUCTION.pdf

https://github.com/jblance/mpp-solar/blob/master/mppsolar/protocols/jk02.py
https://github.com/jblance/jkbms
https://github.com/sshoecraft/jktool/blob/main/jk_info.c
https://github.com/syssi/esphome-jk-bms/blob/main/components/jk_bms_ble/jk_bms_ble.cpp
https://github.com/PurpleAlien/jk-bms_grafana


fix connection abort:
- https://github.com/hbldh/bleak/issues/631 (use bluetoothctl !)
- https://github.com/hbldh/bleak/issues/666

"""
import asyncio
import time
from collections import defaultdict
from typing import List, Callable, Dict, Optional, Tuple

from bmslib.bms import BmsSample, DeviceInfo
from bmslib.bt import BtBms, enumerate_services
from bmslib.util import to_hex_str


def calc_crc(message_bytes):
    return sum(message_bytes) & 0xFF


def read_str(buf, offset, encoding='utf-8'):
    # errors='replace': some JK firmwares put non-UTF8 bytes in the device-info
    # block (#349: 0xbf), which must not crash the sampling loop over a version string.
    return buf[offset:buf.index(0x00, offset)].decode(encoding=encoding, errors='replace')


def _jk_command(address, value: list = ()):
    n = len(value)
    assert n <= 13, "val %s too long" % value
    frame = bytes([0xAA, 0x55, 0x90, 0xEB, address, n])
    frame += bytes(value)
    frame += bytes([0] * (13 - n))
    frame += bytes([calc_crc(frame)])
    return frame


MIN_RESPONSE_SIZE = 300

HEADER = bytes([0x55, 0xAA, 0xEB, 0x90])
FRAME_SIZE = MIN_RESPONSE_SIZE  # a JK response frame is exactly 300 B, CRC in the last one

# Response types the BMS actually sends: 0x01 settings, 0x02 status, 0x03 device info.
# calc_crc is an 8-bit sum, so a HEADER-looking sequence in junk has a ~1/256 chance
# of its 300-byte window passing the checksum. Accepting such a window would silently
# swallow the real frame behind it, so also require a known type byte.
RESPONSE_TYPES = frozenset((0x01, 0x02, 0x03))


def feed_frames(buf: bytearray, chunk: bytes) -> Tuple[List[bytes], int, List[bytes]]:
    """Accumulate ``chunk`` into ``buf`` and return
    ``(complete frames, discarded junk bytes, corrupt frames)``.
    Returned frames are CRC-checked and removed from ``buf``.

    The JK BLE endpoint is a UART bridge: notify packets carry a raw byte stream
    and do *not* respect frame boundaries. One packet can hold the tail of one
    frame and the head of the next (#377: a 384 B buffer = frame 0x01 + the first
    84 B of frame 0x03), and some firmwares splice non-protocol junk into the
    stream (an 'AT\\r\\n' flood, #370, or an echo of the host command frame).
    So we resync on HEADER and consume frame-by-frame instead of assuming that
    a notify packet starts a frame and that the buffer holds at most one.
    """
    buf.extend(chunk)
    frames: List[bytes] = []
    corrupt: List[bytes] = []
    dropped = 0
    keep = len(HEADER) - 1  # a header may straddle two notify packets

    while True:
        if len(buf) < len(HEADER):
            break

        idx = buf.find(HEADER)
        if idx < 0:
            dropped += len(buf) - keep
            del buf[:-keep]
            break
        if idx > 0:
            dropped += idx
            del buf[:idx]

        if len(buf) < FRAME_SIZE:
            break  # header-aligned but incomplete, wait for more packets

        frame = bytes(buf[:FRAME_SIZE])
        if (calc_crc(frame[:-1]) == frame[-1] and frame[4] in RESPONSE_TYPES
                and frame.find(HEADER, 1) < 0):
            del buf[:FRAME_SIZE]
            frames.append(frame)
            continue

        # Either a bad CRC (a real frame arrived corrupt), a header-shaped
        # sequence in junk whose window happened to pass the 8-bit sum, or a
        # frame with a notify packet dropped mid-way (#391): the window then
        # ends with the head of the *next* frame, header included, and passes
        # the sum once in 256. Neither is fixable by more data, and consuming
        # FRAME_SIZE here would eat the real frame behind it. Resync on the
        # next header instead.
        corrupt.append(frame)
        nxt = buf.find(HEADER, len(HEADER))
        if nxt < 0:
            del buf[:-keep]
            break
        del buf[:nxt]

    return frames, dropped, corrupt


class JKBt(BtBms):
    SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
    CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

    TIMEOUT = 12

    SOC_NOT_FULL_YET = 99.0  # when the gauge reaches 100% but no OV yet
    TEMPERATURE_STEP = 0.1
    TEMPERATURE_SMOOTH = 30

    # Throttle window for non-protocol junk on the notify characteristic. Some
    # firmwares (e.g. JK-PB inverters, #370) flood 'AT\r\n' on the shared UART;
    # logging one ERROR + full buffer per packet rolls the log over before the
    # real disconnect is captured and hammers I/O. Collapse to one line / window.
    JUNK_LOG_PERIOD = 30

    def __init__(self, address, keep_alive=True, **kwargs):
        super().__init__(address, keep_alive=keep_alive, **kwargs)
        if kwargs.get('pin'):
            self.logger.warning('JK usually does not use a pairing PIN')
        self._buffer = bytearray()
        self._junk_count = 0
        self._junk_log_t = 0.0
        self._resp_table: Dict[int, Tuple[bytearray, float]] = {}
        self.num_cells = None
        self._callbacks: Dict[int, List[Callable[[bytes], None]]] = defaultdict(List)
        self.char_handle_notify = None
        self.char_handle_write = None
        self.is_new_11fw_32s = None  # https://github.com/syssi/esphome-jk-bms/blob/main/esp32-ble-example.yaml#L6
        self._has_float_charger = None # used for the `float_charge` switch

    def _notification_handler(self, _sender, data):
        data = bytes(data)
        self.logger.debug("bms msg(%d) (buf%d): %s\n", len(data), len(self._buffer), to_hex_str(data))

        # Supersedes the #373 IndexError guard: feed_frames only ever indexes a
        # complete FRAME_SIZE window, so a short buffer can no longer overrun.
        frames, dropped, corrupt = feed_frames(self._buffer, data)

        # feed_frames consumes or trims on every iteration, so it must return with
        # less than one frame still buffered: either a header-aligned partial frame
        # or the <= 3 byte straddle reserve. More than that means the resync logic
        # stopped consuming, and keeping the buffer would stall decoding forever -
        # resync, and make the bug visible instead of growing quietly.
        # Do NOT key this on `dropped` (#392): junk is already deleted by the time
        # it is counted, so what is left over is the partial frame we still need,
        # and dropping it turns a recoverable split frame into a lost one (#377).
        if len(self._buffer) >= FRAME_SIZE:
            self.logger.error("%s framing invariant broken, %d byte(s) buffered after "
                              "parsing (>= one %d B frame), resyncing - please report",
                              self.name, len(self._buffer), FRAME_SIZE)
            self._buffer.clear()

        for frame in corrupt:
            # A real frame that arrived corrupted - rare, keep visible.
            self.logger.error("%s crc check failed, discarding frame 0x%02x: %s...",
                              self.name, frame[4], to_hex_str(frame[:32]))

        if dropped:
            # Non-protocol junk on the notify char, e.g. a JK-PB inverter flooding
            # 'AT\r\n' on the shared UART (#370). Throttle so a flood cannot roll
            # the log over before the real disconnect is captured.
            now = time.time()
            self._junk_count += dropped
            if now - self._junk_log_t >= self.JUNK_LOG_PERIOD:
                self.logger.warning(
                    "%s discarded %d junk byte(s) between frames in %.0fs "
                    "(e.g. JK-PB AT-flood #370); last %d bytes: %.40s",
                    self.name, self._junk_count,
                    (now - self._junk_log_t) if self._junk_log_t else 0,
                    len(data), to_hex_str(data))
                self._junk_log_t = now
                self._junk_count = 0

        for frame in frames:
            self._junk_count = 0
            self._decode_msg(bytearray(frame))

    def _status_frame_implausible(self, buf: bytearray) -> Optional[str]:
        """Physical sanity check on a 0x02 status frame; returns the reason or None.

        calc_crc is an 8-bit sum, so a frame spliced from two status frames after a
        dropped notify packet (a busy ESPHome proxy, #391) still passes the checksum
        once in 256. Every field after the splice is then read at the wrong offset,
        e.g. 1,216,000 V and 130 GW published to HA. The layout must be known.
        """
        if self.is_new_11fw_32s is None:
            return None
        offset = 32 if self.is_new_11fw_32s else 0
        slots = 32 if self.is_new_11fw_32s else 24
        cells = [int.from_bytes(buf[6 + 2 * i:8 + 2 * i], 'little') for i in range(slots)]
        live = [mv for mv in cells if mv]
        if any(not 500 <= mv <= 5000 for mv in live):
            return 'cell voltage out of range: %s' % live
        if self.num_cells and len(live) > self.num_cells:
            return '%d cell voltages for a %d cell pack' % (len(live), self.num_cells)
        voltage = int.from_bytes(buf[118 + offset:122 + offset], 'little') * 1e-3
        cell_sum = sum(live) * 1e-3
        # a cell reading 0 (broken sense wire) must not drop the whole pack from HA
        if abs(voltage - cell_sum) > 0.15 * cell_sum + 1.0:
            return 'pack %.2f V vs cell sum %.2f V' % (voltage, cell_sum)
        current = int.from_bytes(buf[126 + offset:130 + offset], 'little', signed=True) * 1e-3
        if abs(current) > 2000:
            return 'current %.0f A' % current
        # fields behind the splice: the next frame's cell voltages land here
        for i in (130, 132):
            t = int.from_bytes(buf[i + offset:i + offset + 2], 'little', signed=True)
            if t != -2000 and not -500 <= t <= 1500:
                return 'temperature %.1f C' % (t / 10)
        if buf[141 + offset] > 100:
            return 'SOC %d %%' % buf[141 + offset]
        return None

    def _decode_msg(self, buf: bytearray):
        resp_type = buf[4]
        self.logger.debug('got response %d (len%d)', resp_type, len(buf))
        if resp_type == 0x02:
            why = self._status_frame_implausible(buf)
            if why:
                self.logger.error("%s implausible status frame, discarding (%s): %s...",
                                  self.name, why, to_hex_str(buf[:32]))
                return
        self._resp_table[resp_type] = buf, time.time()
        self._fetch_futures.set_result(resp_type, buf[:])
        callbacks = self._callbacks.get(resp_type, None)
        if callbacks:
            for cb in callbacks:
                cb(buf)

    async def _ensure_services_discovered(self, timeout=4):
        # JK v19 firmware (e.g. 19.24, #346/#306/#310) sometimes returns from
        # connect() with client.services == [] — discovery silently failed, or
        # BlueZ handed back a stale empty cache. get_service() then raises the
        # cryptic "service ffe0 not found (have [])". Wait a beat (covers a
        # late-resolving discovery on bleak 2.x), force a re-discovery on older
        # bleak builds that still expose get_services(), then surface a clear
        # error with the workaround.
        deadline = time.monotonic() + timeout
        while True:
            if list(self.client.services):
                return
            get_svc = getattr(self.client, 'get_services', None)
            if get_svc is not None:
                try:
                    await get_svc()
                except Exception as e:
                    self.logger.debug("%s get_services() retry failed: %s", self.name, e)
                if list(self.client.services):
                    return
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "%s: GATT service discovery returned no services for %s. "
                    "Known JK v19 firmware issue / stale BlueZ cache. "
                    "Try `bluetoothctl remove %s` (or restart the bluetooth service), "
                    "then reconnect. If the BMS still won't connect, flashing JK firmware "
                    "19.05 or 19.10 has worked for others (see issue #306)."
                    % (self.name, self.address, self.address)
                )
            await asyncio.sleep(0.5)

    async def connect(self, timeout=20):
        """
        Connecting JK with bluetooth appears to require a prior bluetooth scan and discovery, otherwise the connectiong fails with
        `[org.bluez.Error.Failed] Software caused connection abort`. Maybe the scan triggers some wake up?
        :param timeout:`
        :return:
        """

        try:
            await super().connect(timeout=timeout / 2)
        except Exception as e:
            self.logger.info("%s normal connect failed (%s), connecting with scanner (adapter: %s)", self.name,
                             str(e) or type(e), self._adapter or 'default')
            await self._connect_with_scanner(timeout=timeout)

        await self._ensure_services_discovered(timeout=4)

        service = self.get_service(self.SERVICE_UUID)
        self.char_handle_write = (self.find_char(self.CHAR_UUID, 'write', service=service) or
                                  self.find_char(self.CHAR_UUID, 'write-without-response', service=service))

        if self.char_handle_write is None:
            self.logger.warning("%s Write Characteristic %s not found, enum services:", self.name, self.CHAR_UUID)
            await enumerate_services(self.client, self.logger)

        if self.char_handle_write and hasattr(self.char_handle_write,
                                              'handle') and self.char_handle_write.handle == 0x03:
            # from https://github.com/syssi/esphome-jk-bms/blob/main/components/jk_bms_ble/jk_bms_ble.cpp#L197C17-L197C17
            self.char_handle_notify = self.find_char(0x05, 'notify')

        if not self.char_handle_notify:
            # there might be 2 chars with same uuid (weird?), one for notify/read and one for write
            # https://github.com/fl4p/batmon-ha/issues/83
            self.char_handle_notify = self.find_char(self.CHAR_UUID, 'notify')

        self.logger.debug('char_handle_notify=%s, char_handle_write=%s', self.char_handle_notify,
                          self.char_handle_write)

        await self.start_notify(self.char_handle_notify, self._notification_handler)

        await self._q(cmd=0x97, resp=0x03)  # device info
        await self._q(cmd=0x96, resp=(0x02, 0x01))  # device state (resp 0x01 & 0x02)
        # after these 2 commands the bms will continuously send 0x02-type messages

        buf, _ = self._resp_table[0x01]
        self.num_cells = buf[114]
        assert 0 < self.num_cells <= 24, "num_cells unexpected %s" % self.num_cells

    async def disconnect(self):
        await self.client.stop_notify(self.char_handle_notify)
        await super().disconnect()

    async def _q(self, cmd, resp):
        await asyncio.sleep(.1)
        with await self._fetch_futures.acquire_timeout(resp, timeout=self.TIMEOUT / 2):
            frame = _jk_command(cmd, [])
            self.logger.debug("write %s", frame)
            await self.client.write_gatt_char(self.char_handle_write, data=frame)
            return await self._fetch_futures.wait_for(resp, self.TIMEOUT)

    async def _write(self, address, value):
        frame = _jk_command(address, value)
        self.logger.debug("write> %s", frame)
        await self.client.write_gatt_char(self.char_handle_write, data=frame)

    async def fetch_device_info(self):
        # https://github.com/jblance/mpp-solar/blob/master/mppsolar/protocols/jkabstractprotocol.py
        # https://github.com/syssi/esphome-jk-bms/blob/main/components/jk_bms_ble/jk_bms_ble.cpp#L1152
        buf, _ = self._resp_table[0x03]
        psk = read_str(buf, 6 + 16 + 8 + 16 + 40 + 11)
        if psk and self._has_float_charger is None:
            self.logger.info("PSK = '%s' (Note that anyone within BLE range can read this!)", psk)

        di = DeviceInfo(mnf="JK",
                        model=read_str(buf, 6),
                        hw_version=read_str(buf, 6 + 16),
                        sw_version=read_str(buf, 6 + 16 + 8),
                        name=read_str(buf, 6 + 16 + 8 + 16),
                        sn=read_str(buf, 6 + 16 + 8 + 16 + 40),
                        )
        self._has_float_charger = ('PB2A16S' in di.model) or ('PB1A16S' in di.model)
        return di

    async def has_float_charger(self):
        if self._has_float_charger is None:
            await self.fetch_device_info()
        return self._has_float_charger

    def _decode_sample(self, buf: bytearray, t_buf: float, has_float_charger: bool) -> BmsSample:
        buf_set, t_set = self._resp_table[0x01]

        offset = 0
        if self.is_new_11fw_32s is None:
            self.is_new_11fw_32s = True

        if self.is_new_11fw_32s:
            offset = 32
            self.logger.debug('New 11.x firmware, offset=%s', offset)

        i16 = lambda i: int.from_bytes(buf[i:(i + 2)], byteorder='little', signed=True)
        u32 = lambda i: int.from_bytes(buf[i:(i + 4)], byteorder='little', signed=False)
        f32u = lambda i: u32(i) * 1e-3
        f32s = lambda i: int.from_bytes(buf[i:(i + 4)], byteorder='little', signed=True) * 1e-3

        temp = lambda x: float('nan') if x == -2000 else (x / 10)

        temperatures = [temp(i16(130 + offset)), temp(i16(132 + offset))]
        if self.is_new_11fw_32s:
            temperatures += [temp(i16(224 + offset)), temp(i16(226 + offset))]

        # SOH and BMS-internal aged capacity only meaningful on 11.x firmware.
        # On legacy 24s firmware, offset 146 just mirrors the nominal capacity
        # and there's no SOH byte, so leave both fields nan.
        soh = float('nan')
        aged_capacity = float('nan')
        if self.is_new_11fw_32s:
            soh = float(buf[158 + offset])  # SOH at 158+offset = 190, 1 byte, %
            aged_capacity = f32u(146 + offset)  # BMS-computed effective Ah

        charge_remaining = f32u(142 + offset)  # "remaining capacity"
        # SOC byte at 141 is 1% resolution. Recover sub-1% precision by
        # recomputing charge_remaining / aged_capacity — both are BMS-internal
        # values, so the ratio reproduces the BMS-displayed SOC at full
        # precision (#369). Pass as float so BmsSample doesn't override it
        # against the user-configured capacity from the settings frame, which
        # would mis-scale SOC on aged 11.x packs (#365).
        if aged_capacity > 0 and charge_remaining > 0:
            soc = round(charge_remaining / aged_capacity * 100, 2)
        else:
            soc = int(buf[141 + offset])  # int → BmsSample recomputes for legacy

        return BmsSample(
            voltage=f32u(118 + offset),
            current=-f32s(126 + offset),
            soc=soc,

            total_charge_throughput=f32u(154 + offset),  # lifetime ∫|I|dt, Ah
            # capacity: user-configured pack capacity from the settings frame.
            # The cell-info frame at offset 146+offset is an internal BMS-aged
            # value that diverges from the configured Ah on 11.x firmware (#365).
            capacity=int.from_bytes(buf_set[130:134], byteorder='little', signed=False) * 1e-3,
            charge=charge_remaining,
            soh=soh,
            aged_capacity=aged_capacity,

            temperatures=temperatures,
            mos_temperature=i16((112 if self.is_new_11fw_32s else 134) + offset) / 10,
            balance_current=i16(138 + offset) / 1000,

            # 146 charge_full (see above)
            num_cycles=u32(150 + offset),
            switches=dict(
                charge=bool(buf_set[118]),
                discharge=bool(buf_set[122]),
                balance=bool(buf_set[126]),
                **(dict(float_charge=bool(buf_set[283] & 2)) if has_float_charger else {}),
            ),
            #  #buf[166 + offset]),  charge FET state
            # buf[167 + offset]), discharge FET state
            uptime=float(u32(162 + offset)),  # seconds
            timestamp=t_buf,
        )

    async def fetch(self, wait=True) -> BmsSample:

        """
        Decode JK02
        references
        * https://github.com/syssi/esphome-jk-bms/blob/main/components/jk_bms_ble/jk_bms_ble.cpp#L360
        * https://github.com/jblance/mpp-solar/blob/master/mppsolar/protocols/jk02.py
        """

        if wait:
            with await self._fetch_futures.acquire_timeout(0x02, timeout=self.TIMEOUT / 2):
                await self._fetch_futures.wait_for(0x02, self.TIMEOUT)

        if 0x01 not in self._resp_table:
            await self._q(cmd=0x96, resp=0x01)  # query settings

        if self.is_new_11fw_32s is None:
            di = None
            try:
                di = await self.fetch_device_info()
                self.is_new_11fw_32s = int(di.sw_version.split('.')[0]) >= 11
                self.logger.info('%s SW ver %s detected frame ver: %s', self, di.sw_version,
                                 "32s (fw>=11)" if self.is_new_11fw_32s else "24s (fw<11)")
            except Exception as e:
                self.logger.info("Unrecognized SW version %s", di)

        buf, t_buf = self._resp_table[0x02]
        has_float_charger = await self.has_float_charger()
        return self._decode_sample(buf, t_buf, has_float_charger=has_float_charger)

    async def subscribe(self, callback: Callable[[BmsSample], None]):
        self._callbacks[0x02].append(lambda buf: callback(
            self._decode_sample(buf, t_buf=time.time(), has_float_charger=bool(self._has_float_charger))))

    async def fetch_voltages(self):
        """
        :return: list of cell voltages in mV
        """
        if self.num_cells is None:
            raise Exception("num_cells not set")
        buf, t_buf = self._resp_table[0x02]
        voltages = [int.from_bytes(buf[(6 + i * 2):(6 + i * 2 + 2)], byteorder='little') for i in
                    range(self.num_cells)]
        return voltages

    async def set_switch(self, switch: str, state: bool):
        # from https://github.com/syssi/esphome-jk-bms/blob/4079c22eaa40786ffa0cabd45d0d98326a1fdd29/components/jk_bms_ble/switch/__init__.py
        addresses = dict(
            charge=0x1D,
            discharge=0x1E,
            balance=0x1F
        )

        if await self.has_float_charger():
            addresses['float_charge'] = 0x30

        await self._write(addresses[switch], [0x1 if state else 0x0, 0, 0, 0])
        await asyncio.sleep(.2)  # wait a bit before triggering settings fetch
        self._resp_table.pop(0x01, None)  # invalidate settings frame which stores switch states
        # await asyncio.sleep(0.2)  # not sure if this is needed

    def supports_set_soc(self) -> bool:
        # register 0x6E exists in the JK02_32S protocol only (fw >= 11), see syssi/esphome-jk-bms
        # components/jk_bms_ble/number/__init__.py CONF_SOC_CALIBRATION (#144)
        return bool(self.is_new_11fw_32s)

    async def set_soc(self, soc: float):
        if not self.is_new_11fw_32s:
            raise NotImplementedError("JK SOC calibration needs the 32s protocol (firmware >= 11)")
        if not (0 <= soc <= 100):
            raise ValueError("soc must be within 0..100, got %r" % (soc,))
        await self._write(0x6E, [int(round(soc))])  # 1-byte register, percent
        await asyncio.sleep(.2)
        self._resp_table.pop(0x01, None)  # settings frame is stale now

    # ---- Generic configuration registers ----------------------------------------
    # Mirrors syssi/esphome-jk-bms's `number:` platform (components/jk_bms_ble/number/__init__.py),
    # so every parameter configurable from the ESPHome YAML can be written here too.
    #
    # name -> (register_24s, register_32s, factor, length_bytes)
    # A register of 0x00 means "not supported on this protocol variant" (JK02_24S,
    # firmware < 11, vs. JK02_32S, firmware >= 11).
    CONFIG_NUMBERS: Dict[str, Tuple[int, int, float, int]] = {
        'smart_sleep_voltage': (0x01, 0x01, 1000.0, 1),
        'cell_voltage_undervoltage_protection': (0x02, 0x02, 1000.0, 4),
        'cell_voltage_undervoltage_recovery': (0x03, 0x03, 1000.0, 4),
        'cell_voltage_overvoltage_protection': (0x04, 0x04, 1000.0, 4),
        'cell_voltage_overvoltage_recovery': (0x05, 0x05, 1000.0, 4),
        'balance_trigger_voltage': (0x06, 0x06, 1000.0, 4),
        'cell_soc100_voltage': (0x07, 0x07, 1000.0, 4),
        'cell_soc0_voltage': (0x08, 0x08, 1000.0, 4),
        'cell_request_charge_voltage': (0x09, 0x09, 1000.0, 4),
        'cell_request_float_voltage': (0x0A, 0x0A, 1000.0, 4),
        'power_off_voltage': (0x0B, 0x0B, 1000.0, 4),
        'max_charge_current': (0x0C, 0x0C, 1000.0, 4),
        'charge_overcurrent_protection_delay': (0x0D, 0x0D, 1.0, 4),
        'charge_overcurrent_protection_recovery_time': (0x0E, 0x0E, 1.0, 4),
        'max_discharge_current': (0x0F, 0x0F, 1000.0, 4),
        'discharge_overcurrent_protection_delay': (0x10, 0x10, 1.0, 4),
        'discharge_overcurrent_protection_recovery_time': (0x11, 0x11, 1.0, 4),
        'short_circuit_protection_recovery_time': (0x12, 0x12, 1.0, 4),
        'max_balance_current': (0x13, 0x13, 1000.0, 4),
        'charge_overtemperature_protection': (0x14, 0x14, 10.0, 4),
        'charge_overtemperature_protection_recovery': (0x15, 0x15, 10.0, 4),
        'discharge_overtemperature_protection': (0x16, 0x16, 10.0, 4),
        'discharge_overtemperature_protection_recovery': (0x17, 0x17, 10.0, 4),
        'charge_undertemperature_protection': (0x18, 0x18, 10.0, 4),
        'charge_undertemperature_protection_recovery': (0x19, 0x19, 10.0, 4),
        'mosfet_overtemperature_protection': (0x1A, 0x1A, 10.0, 4),
        'mosfet_overtemperature_protection_recovery': (0x1B, 0x1B, 10.0, 4),
        'cell_count': (0x1C, 0x1C, 1.0, 4),
        'total_battery_capacity': (0x20, 0x20, 1000.0, 4),
        'voltage_calibration': (0x21, 0x64, 1000.0, 4),
        'short_circuit_protection_delay': (0x25, 0x21, 1.0, 4),
        'balancing_start_voltage': (0x26, 0x22, 1000.0, 4),
        'current_calibration': (0x24, 0x67, 1000.0, 4),
        'discharge_precharge_time': (0x00, 0x25, 1.0, 4),
        'heating_start_temperature': (0x00, 0x37, 1.0, 1),
        'heating_stop_temperature': (0x00, 0x38, 1.0, 1),
        'smart_sleep_delay': (0x00, 0x39, 1.0, 1),
        'discharge_undertemperature_protection': (0x00, 0x3A, 1.0, 1),
        'discharge_undertemperature_protection_recovery': (0x00, 0x3B, 1.0, 1),
        'soc_calibration': (0x00, 0x6E, 1.0, 1),
        'soh_calibration': (0x00, 0x6F, 1.0, 1),
        'cell_request_charge_voltage_time': (0x00, 0xB3, 10.0, 1),
        'cell_request_float_voltage_time': (0x00, 0xB4, 10.0, 1),
        'emergency_duration': (0x00, 0xB5, 1.0, 1),
        're_bulk_soc': (0x00, 0xB7, 1.0, 1),
    }

    # Registers <= this are laid out linearly in the 0x01 settings frame at
    # offset = 6 + (register-1)*4, so their current value can be decoded back out
    # of it (verified against esphome-jk-bms's decode_jk02_settings_()). Above it,
    # firmwares reuse that space for other data (per-cell wire resistance tables,
    # bitmask flags, ...), so those registers can be written but not read back here
    # - same limitation the ESPHome component has for its "unnamed"/no-state fields.
    _READBACK_MAX_REGISTER = 0x22

    # Registers whose value can be negative (temperatures below 0 C).
    _SIGNED_CONFIG_NUMBERS = frozenset((
        'charge_undertemperature_protection',
        'charge_undertemperature_protection_recovery',
    ))

    def _config_register(self, name: str) -> int:
        reg24, reg32, _factor, _len = self.CONFIG_NUMBERS[name]
        return reg32 if self.is_new_11fw_32s else reg24

    def supports_config_number(self, name: str) -> bool:
        if name not in self.CONFIG_NUMBERS or self.is_new_11fw_32s is None:
            return False
        return bool(self._config_register(name))

    async def set_config_number(self, name: str, value: float):
        """Write any parameter in CONFIG_NUMBERS, e.g. set_config_number(
        'cell_voltage_overvoltage_protection', 3.65). Mirrors the write path of
        esphome-jk-bms's number platform (JkNumber::control -> write_register)."""
        if name not in self.CONFIG_NUMBERS:
            raise KeyError("unknown JK config number %r" % name)
        reg = self._config_register(name)
        if not reg:
            raise NotImplementedError(
                "%s is not available on this JK protocol variant (%s)" % (
                    name, "32S/fw>=11" if self.is_new_11fw_32s else "24S/fw<11"))
        _reg24, _reg32, factor, length = self.CONFIG_NUMBERS[name]
        raw = int(round(value * factor)) & ((1 << (8 * length)) - 1)  # two's complement
        payload = list(raw.to_bytes(length, byteorder='little'))
        await self._write(reg, payload)
        await asyncio.sleep(.2)  # wait a bit before triggering settings fetch
        self._resp_table.pop(0x01, None)  # invalidate settings frame which stores it

    def get_config_numbers(self) -> Dict[str, float]:
        """Best-effort decode of the current value of every CONFIG_NUMBERS entry
        from the last-seen settings (0x01) frame. Entries whose register is above
        _READBACK_MAX_REGISTER are write-only (see comment there) and are omitted."""
        buf_set, _t = self._resp_table.get(0x01, (None, 0))
        if buf_set is None or self.is_new_11fw_32s is None:
            return {}
        out: Dict[str, float] = {}
        for name in self.CONFIG_NUMBERS:
            reg = self._config_register(name)
            if not reg or reg > self._READBACK_MAX_REGISTER:
                continue
            _reg24, _reg32, factor, length = self.CONFIG_NUMBERS[name]
            offset = 6 + (reg - 1) * 4
            if offset + length > len(buf_set):
                continue
            raw = int.from_bytes(buf_set[offset:offset + length], byteorder='little',
                                 signed=(name in self._SIGNED_CONFIG_NUMBERS))
            out[name] = raw / factor
        return out

    def get_wire_resistances(self) -> Optional[List[float]]:
        """Per-cell internal resistance (Ohm), one value per cell - this is what
        esphome-jk-bms's `cell_resistance_1..N` sensors actually show, decoded
        live from the periodic cell-info (0x02) frame (same frame cell voltages
        come from). Read-only, updates every sample.

        Not to be confused with the settings-frame "connector wire resistance"
        *calibration* registers (0x01 frame): those are a manual compensation
        value the vendor app writes after a one-key calibration procedure and
        otherwise sit at 0 - a different field with a similar name."""
        if not self.num_cells:
            return None
        buf, _t = self._resp_table.get(0x02, (None, 0))
        if buf is None or self.is_new_11fw_32s is None:
            return None
        # esphome-jk-bms decode_jk02_cell_info_(): resistance table starts at
        # byte 64 (+16 more on 32S/fw>=11 firmware - NOT +32, that constant is
        # for a different frame; confirmed against upstream source), 2 bytes
        # per cell, unsigned, factor 0.001.
        base = 64 + (16 if self.is_new_11fw_32s else 0)
        out = []
        for i in range(self.num_cells):
            offset = base + i * 2
            if offset + 2 > len(buf):
                break
            raw = int.from_bytes(buf[offset:offset + 2], byteorder='little', signed=False)
            out.append(raw / 1000)
        return out or None

    def debug_data(self):
        return dict(resp=self._resp_table, char_w=self.char_handle_write, char_r=self.char_handle_notify)



async def main():
    # _jk_command(0x96)

    # await bmslib.bt.bt_discovery(logger=get_logger())
    mac_address = 'F21958DF-E949-4D43-B12B-0020365C428A'  # caravan
    # mac_address = '46A9A7A1-D6C6-59C5-52D0-79EC8C77F4D2'  # bat100ah
    mac_address = 'BB92A45B-ABA1-2EA8-1BD3-DA140771C79D'  # caravan (intel)

    bms = JKBt(mac_address, name='jk', verbose_log=False)
    async with bms:
        while True:
            s = await bms.fetch(wait=True)
            # print(s, 'I_bal=', s.balance_current, await bms.fetch_voltages())
            print(s.switches)

            b = not s.switches.get("charge")

            await bms.set_switch("charge", b)

            s = await bms.fetch()
            print(s.switches)

            if s.switches.get("charge") != b:
                print('error', s)

            # new_state = not s.switches['charge']
            # await bms.set_switch('charge', new_state)
            # await self._q(cmd=0x96, resp= 0x01)
            # print('set charge', new_state)
            # await asyncio.sleep(4)
            # s = await bms.fetch(wait=True)
            # print(s)


class JKBt_24s(JKBt):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.is_new_11fw_32s = False


class JKBt_32s(JKBt):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.is_new_11fw_32s = True


if __name__ == '__main__':
    asyncio.run(main())

