"""
Direct HCI BLE scanner via PyUSB for CSR8510 dongle (0x0A12:0x0001).
Bypasses CoreBluetooth entirely — no OS throttling, no deduplication.
"""

import struct
import threading
import time
import usb.core
import usb.util

DONGLE_VENDOR  = 0x0A12
DONGLE_PRODUCT = 0x0001

# ── HCI commands (USB transport: no packet-type indicator byte) ───────────────
# Format: opcode_lo, opcode_hi, param_len, [params...]

HCI_RESET = bytes([0x03, 0x0C, 0x00])

# Enable all classic events + LE Meta event (bit 61 set)
HCI_SET_EVENT_MASK = bytes([
    0x01, 0x0C, 0x08,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F,
])

# Enable LE Advertising Report subevent (bit 1)
HCI_LE_SET_EVENT_MASK = bytes([
    0x01, 0x20, 0x08,
    0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
])

HCI_LE_SET_SCAN_PARAMS = bytes([
    0x0B, 0x20,  # opcode: HCI_LE_Set_Scan_Parameters
    0x07,        # param length
    0x00,        # scan type: passive
    0x10, 0x00,  # scan interval: 16 × 0.625ms = 10ms
    0x10, 0x00,  # scan window:   16 × 0.625ms = 10ms (100% duty cycle)
    0x00,        # own address type: public
    0x00,        # filter policy: accept all
])

HCI_LE_SET_SCAN_ENABLE = bytes([
    0x0C, 0x20,  # opcode: HCI_LE_Set_Scan_Enable
    0x02,        # param length
    0x01,        # enable
    0x00,        # filter duplicates: OFF
])

# ── HCI event constants ───────────────────────────────────────────────────────
HCI_EVT_COMMAND_COMPLETE  = 0x0E
HCI_EVT_LE_META           = 0x3E
HCI_LE_SUBEVENT_ADV_REPORT = 0x02


def _send_cmd(dev, cmd):
    """Send an HCI command via USB control transfer."""
    dev.ctrl_transfer(
        bmRequestType=0x20,  # Class, Interface, Host→Device
        bRequest=0x00,
        wValue=0, wIndex=0,
        data_or_wLength=cmd,
    )


_EP_MAX_PKT = 16  # wMaxPacketSize for CSR8510 interrupt IN endpoint

def _read_event(dev, timeout_ms=200):
    """
    Read a complete HCI event by assembling 16-byte USB packets.
    The CSR8510 interrupt endpoint has wMaxPacketSize=16, so events larger
    than 16 bytes arrive in multiple USB transactions that we reassemble here.
    """
    buf = b""
    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        try:
            chunk = bytes(dev.read(0x81, _EP_MAX_PKT, timeout=100))
            buf += chunk
            if len(buf) >= 2:
                needed = 2 + buf[1]  # event_code(1) + param_len(1) + params
                if len(buf) >= needed:
                    return buf[:needed]
        except usb.core.USBTimeoutError:
            if buf:
                continue   # partial read — keep waiting
            return None
        except usb.core.USBError:
            return None
    return buf if len(buf) >= 2 else None


def _wait_command_complete(dev, label="cmd", retries=15):
    for _ in range(retries):
        data = _read_event(dev, timeout_ms=1000)
        if data and data[0] == HCI_EVT_COMMAND_COMPLETE:
            # Byte layout: event_code(1) param_len(1) num_cmds(1) opcode(2) status(1)
            status = data[5] if len(data) > 5 else 0xFF
            ok = "ok" if status == 0x00 else f"ERR 0x{status:02x}"
            print(f"[hci] {label}: {ok}", flush=True)
            return status == 0x00
    print(f"[hci] {label}: timed out", flush=True)
    return False


def _parse_ad_structures(data):
    """
    Walk AD structures and return manufacturer_data dict {company_id: bytes}.
    Matches the format bleak uses so the existing parse_ibeacon() works unchanged.
    """
    mfr = {}
    i = 0
    while i < len(data):
        length = data[i]
        if length == 0:
            break
        if i + 1 + length > len(data):
            break
        ad_type = data[i + 1]
        ad_payload = data[i + 2: i + 1 + length]
        if ad_type == 0xFF and len(ad_payload) >= 2:   # Manufacturer Specific
            company_id = struct.unpack_from('<H', ad_payload)[0]
            mfr[company_id] = bytes(ad_payload[2:])
        i += 1 + length
    return mfr


def _parse_le_advertising_report(payload, on_advertisement):
    """
    Parse LE Advertising Report subevent (0x02).
    payload starts at subevent_code byte.
    Calls on_advertisement(address, rssi, manufacturer_data) per report.
    """
    if not payload or payload[0] != HCI_LE_SUBEVENT_ADV_REPORT:
        return

    num_reports = payload[1]
    offset = 2

    for _ in range(num_reports):
        if offset + 9 > len(payload):
            break

        # event_type (1) + addr_type (1) + addr (6) + data_len (1)
        addr_bytes = payload[offset + 2: offset + 8]
        address    = ':'.join(f'{b:02X}' for b in reversed(addr_bytes))
        data_len   = payload[offset + 8]
        offset    += 9

        if offset + data_len + 1 > len(payload):
            break

        ad_data = payload[offset: offset + data_len]
        offset += data_len

        rssi = struct.unpack('b', bytes([payload[offset]]))[0]
        offset += 1

        mfr = _parse_ad_structures(ad_data)
        if mfr:
            on_advertisement(address, rssi, mfr)


# ── Public interface ──────────────────────────────────────────────────────────

class HCIScanner:
    """
    Minimal BLE scanner that talks directly to the CSR8510 via USB HCI.
    on_advertisement(address: str, rssi: int, manufacturer_data: dict) is called
    for every advertising packet that contains manufacturer-specific data.
    """

    def __init__(self, on_advertisement):
        self._cb      = on_advertisement
        self._dev     = None
        self._running = False
        self._thread  = None

    def start(self):
        dev = usb.core.find(idVendor=DONGLE_VENDOR, idProduct=DONGLE_PRODUCT)
        if dev is None:
            raise RuntimeError(
                f"CSR dongle not found ({DONGLE_VENDOR:04X}:{DONGLE_PRODUCT:04X}) — "
                "is it plugged in?"
            )

        # Detach any kernel driver that might have claimed the interface
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except usb.core.USBError:
            pass

        dev.set_configuration()
        self._dev = dev

        # Full init sequence — event masks must come before scan commands
        _send_cmd(dev, HCI_RESET)
        _wait_command_complete(dev, "reset")
        time.sleep(0.5)

        _send_cmd(dev, HCI_SET_EVENT_MASK)
        _wait_command_complete(dev, "set_event_mask")

        _send_cmd(dev, HCI_LE_SET_EVENT_MASK)
        _wait_command_complete(dev, "le_set_event_mask")

        # Disable scan first so we can set params (fails if scan is already active)
        _send_cmd(dev, bytes([0x0C, 0x20, 0x02, 0x00, 0x00]))
        _wait_command_complete(dev, "scan_disable")

        _send_cmd(dev, HCI_LE_SET_SCAN_PARAMS)
        _wait_command_complete(dev, "le_set_scan_params")

        _send_cmd(dev, HCI_LE_SET_SCAN_ENABLE)
        _wait_command_complete(dev, "le_set_scan_enable")

        self._running = True
        self._thread  = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print("HCI scanner started on CSR dongle (direct USB, no CoreBluetooth)", flush=True)

    def stop(self):
        self._running = False
        # Disable scanning
        try:
            disable = bytes([0x0C, 0x20, 0x02, 0x00, 0x00])
            _send_cmd(self._dev, disable)
        except Exception:
            pass

    def _read_loop(self):
        while self._running:
            data = _read_event(self._dev, timeout_ms=500)
            if not data or len(data) < 3:
                continue
            if data[0] == HCI_EVT_LE_META:
                _parse_le_advertising_report(data[2:], self._cb)
