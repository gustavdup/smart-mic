"""
Michelin iBeacon Scanner
Scans for iBeacons matching the deployment UUID and serves a live dashboard.
"""

import asyncio
import struct
import threading
import time
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from bleak import BleakScanner

# ── Config ────────────────────────────────────────────────────────────────────

DEPLOYMENT_UUID = "e2c56db5-dffb-48d2-b060-d0f5a71096e0"

# Path-loss exponent (2 = free space, 3-4 = indoors)
PATH_LOSS_N = 2.5

# Forget a beacon after this many seconds without a reading
BEACON_TIMEOUT_S = 30

# ── State (shared between scanner thread and Flask) ───────────────────────────

beacons: dict[str, dict] = {}   # key: "major.minor"
beacons_lock = threading.Lock()
ble_status = {"status": "starting", "mode": None}  # replayed to new clients

# ── Flask app ─────────────────────────────────────────────────────────────────

app = Flask(__name__)
app.config["SECRET_KEY"] = "michelin-scanner"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


@app.route("/")
def index():
    return render_template("index.html")


@socketio.on("connect")
def on_connect():
    # Replay current BLE status, recording status, and all known beacons to the connecting client only
    emit("ble_status", ble_status)
    emit("recording_status", recording_status)
    now = time.time()
    with beacons_lock:
        for b in beacons.values():
            emit("beacon_update", {
                **b,
                "last_seen_iso": time.strftime("%H:%M:%S", time.localtime(b["last_seen"])),
            })
            if now - b["last_seen"] > BEACON_TIMEOUT_S:
                emit("beacon_stale", {"key": b["key"]})


# ── iBeacon parsing ───────────────────────────────────────────────────────────

def parse_ibeacon(manufacturer_data: dict):
    """
    Extract UUID, major, minor, tx_power from Apple iBeacon manufacturer data.
    Returns None if the payload is not a valid iBeacon.
    """
    apple_data = manufacturer_data.get(0x004C)
    if apple_data is None or len(apple_data) < 23:
        return None

    # iBeacon subtype = 0x02, length = 0x15 (21 bytes)
    if apple_data[0] != 0x02 or apple_data[1] != 0x15:
        return None

    uuid_bytes = apple_data[2:18]
    uuid = (
        f"{uuid_bytes[0:4].hex()}-"
        f"{uuid_bytes[4:6].hex()}-"
        f"{uuid_bytes[6:8].hex()}-"
        f"{uuid_bytes[8:10].hex()}-"
        f"{uuid_bytes[10:16].hex()}"
    )

    (major, minor) = struct.unpack(">HH", apple_data[18:22])
    tx_power = struct.unpack("b", bytes([apple_data[22]]))[0]  # signed dBm

    return {"uuid": uuid, "major": major, "minor": minor, "tx_power": tx_power}


def rssi_to_distance(rssi: int, tx_power: int) -> float:
    """Estimate distance in metres using the log-distance path loss model."""
    if rssi == 0:
        return -1.0
    ratio = (tx_power - rssi) / (10.0 * PATH_LOSS_N)
    return round(10 ** ratio, 2)


# ── BLE scanner ───────────────────────────────────────────────────────────────

def on_beacon(key, major, minor, rssi, tx_power, distance, address):
    """Shared handler called by both the HCI and bleak scanner paths."""
    now = time.time()
    print(f"[beacon] {key}  rssi={rssi}  dist={distance}m", flush=True)
    update = {
        "key": key,
        "major": major,
        "minor": minor,
        "rssi": rssi,
        "tx_power": tx_power,
        "distance_m": distance,
        "last_seen": now,
        "address": address,
        "name": "iBeacon",
        "stale": False,
    }
    with beacons_lock:
        beacons[key] = update
    socketio.emit("beacon_update", {**update, "last_seen_iso": time.strftime("%H:%M:%S", time.localtime(now))})


def hci_advertisement_callback(address, rssi, manufacturer_data):
    """Called by HCIScanner for every received advertising packet."""
    parsed = parse_ibeacon(manufacturer_data)
    if parsed is None:
        return
    if parsed["uuid"].lower() != DEPLOYMENT_UUID.lower():
        return
    distance = rssi_to_distance(rssi, parsed["tx_power"])
    key = f"{parsed['major']}.{parsed['minor']}"
    on_beacon(key, parsed["major"], parsed["minor"], rssi, parsed["tx_power"], distance, address)


def bleak_detection_callback(device, advertisement_data):
    """Called by BleakScanner (CoreBluetooth path)."""
    mfr = advertisement_data.manufacturer_data
    if not mfr:
        return
    parsed = parse_ibeacon(mfr)
    if parsed is None:
        return
    if parsed["uuid"].lower() != DEPLOYMENT_UUID.lower():
        return
    rssi = advertisement_data.rssi
    distance = rssi_to_distance(rssi, parsed["tx_power"])
    key = f"{parsed['major']}.{parsed['minor']}"
    on_beacon(key, parsed["major"], parsed["minor"], rssi, parsed["tx_power"], distance, device.address)


async def make_scanner():
    """
    Create a BleakScanner with CBCentralManagerScanOptionAllowDuplicatesKey=True.
    This disables macOS's default deduplication so we get a callback for every
    advertisement, not just the first one per device.
    """
    return BleakScanner(
        detection_callback=bleak_detection_callback,
        cb={"use_bdaddr": False, "allow_duplicates": True},
    )


def try_hci_scanner():
    """
    Attempt to start the direct USB HCI scanner on the CSR dongle.
    Returns the HCIScanner instance on success, None on failure.
    """
    try:
        from hci_scanner import HCIScanner
        hci = HCIScanner(hci_advertisement_callback)
        hci.start()
        ble_status.update({"status": "scanning", "mode": "HCI (USB dongle)"})
        socketio.emit("ble_status", ble_status)
        return hci
    except Exception as e:
        print(f"HCI scanner unavailable ({e}) — falling back to CoreBluetooth", flush=True)
        return None


def hci_stale_loop():
    """Background thread: marks beacons stale when not seen for BEACON_TIMEOUT_S."""
    while True:
        time.sleep(1)
        now = time.time()
        with beacons_lock:
            for k, b in beacons.items():
                if (now - b["last_seen"] > BEACON_TIMEOUT_S) and not b.get("stale"):
                    b["stale"] = True
                    socketio.emit("beacon_stale", {"key": k})


async def run_bleak_scanner():
    """CoreBluetooth fallback path (used when HCI dongle is not available)."""
    scanner = None
    while True:
        try:
            scanner = await make_scanner()
            await scanner.start()
            break
        except Exception as e:
            print(f"BLE init failed ({e}) — retrying in 3s…", flush=True)
            ble_status.update({"status": "error", "message": str(e)})
            socketio.emit("ble_status", ble_status)
            await asyncio.sleep(3)

    print("CoreBluetooth scanner started — looking for UUID:", DEPLOYMENT_UUID, flush=True)
    ble_status.update({"status": "scanning", "mode": "CoreBluetooth (built-in)"})
    socketio.emit("ble_status", ble_status)

    while True:
        await asyncio.sleep(1)

        now = time.time()
        with beacons_lock:
            for k, b in beacons.items():
                if (now - b["last_seen"] > BEACON_TIMEOUT_S) and not b.get("stale"):
                    b["stale"] = True
                    socketio.emit("beacon_stale", {"key": k})

        try:
            await scanner.stop()
            scanner = await make_scanner()
            await scanner.start()
        except Exception as e:
            print(f"Scanner cycle error ({e})", flush=True)


def scanner_thread():
    # Use CoreBluetooth (built-in Mac Bluetooth)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_bleak_scanner())


# ── ESP32 HTTP endpoint ───────────────────────────────────────────────────────

@app.route("/api/beacon", methods=["POST"])
def api_beacon():
    data = request.get_json(silent=True)
    if not data:
        return jsonify({"error": "no json"}), 400
    try:
        major    = int(data["major"])
        minor    = int(data["minor"])
        rssi     = int(data["rssi"])
        tx_power = int(data["tx_power"])
        address  = str(data.get("address", "esp32"))
    except (KeyError, ValueError) as e:
        return jsonify({"error": str(e)}), 400
    distance = rssi_to_distance(rssi, tx_power)
    key = f"{major}.{minor}"
    on_beacon(key, major, minor, rssi, tx_power, distance, address)
    return jsonify({"ok": True})


# ── ESP32 recording status endpoint ──────────────────────────────────────────

recording_status = {"state": "idle", "file": None}  # idle | recording | uploading

@app.route("/api/status", methods=["POST"])
def api_status():
    data = request.get_json(silent=True)
    if not data:
        return jsonify({"error": "no json"}), 400
    recording_status.update({
        "state": data.get("state", "idle"),
        "file":  data.get("file"),
    })
    socketio.emit("recording_status", recording_status)
    return jsonify({"ok": True})


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    t = threading.Thread(target=scanner_thread, daemon=True)
    t.start()

    print("Dashboard at http://127.0.0.1:5001")
    socketio.run(app, host="0.0.0.0", port=5001, debug=False, allow_unsafe_werkzeug=True)
