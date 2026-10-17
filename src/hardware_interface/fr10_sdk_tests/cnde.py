"""CNDE output-only client, manual 3.9.3 chapter 6, little-endian frames."""
import json
import math
import socket
import struct
import time
from pathlib import Path

FIELD_TYPES = {
    "actual_joint_pos": ("DOUBLE_6", "6d"),
    "actual_TCP_pos": ("DOUBLE_6", "6d"),
    "actual_joint_vel": ("DOUBLE_6", "6d"),
    "robot_mode": ("UINT8", "B"),
    "run_up_time": ("DOUBLE", "d"),
}


def frame(kind, payload=b"", count=0):
    if not 0 <= kind <= 8 or len(payload) > 4096:
        raise ValueError("Invalid CNDE frame")
    return struct.pack("<HBBH", 0x5A5A, count % 256, kind, len(payload)) + payload + b"\xa5\xa5"


def parse(data):
    if len(data) < 8:
        raise ValueError("CNDE frame shorter than header/trailer")
    head, count, kind, size = struct.unpack_from("<HBBH", data)
    if head != 0x5A5A or data[-2:] != b"\xa5\xa5" or len(data) != size + 8:
        raise ValueError("CNDE header, length or trailer mismatch")
    return count, kind, data[6:-2]


def validate(config):
    c = config["cnde"]
    if type(c["cycle_ms"]) is not int or not 1 <= c["cycle_ms"] <= 200:
        raise ValueError("CNDE cycle_ms must be an integer in [1,200]")
    if type(c["port"]) is not int or not 1 <= c["port"] <= 65535:
        raise ValueError("Invalid CNDE UDP port")
    if not c["fields"] or len(set(c["fields"])) != len(c["fields"]):
        raise ValueError("CNDE fields must be nonempty and unique")
    for field in c["fields"]:
        if field not in FIELD_TYPES:
            raise ValueError(f"Unsupported decoder for field: {field}")
    duration = config["duration_s"]
    if not isinstance(duration, (int, float)) or not math.isfinite(duration) or not 0.04 <= duration <= 10:
        raise ValueError("Invalid observation duration")


def run(config, output, socket_factory=socket.socket):
    validate(config)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    fields = config["cnde"]["fields"]
    types = [FIELD_TYPES[f][0] for f in fields]
    fmt = "<" + "".join(FIELD_TYPES[f][1] for f in fields)
    result = {"case": "cnde", "status": "FAILED", "configuration": config,
              "cleanup_confirmed": False, "packets": 0, "invalid_packets": 0,
              "duplicates": 0, "estimated_lost_packets": 0,
              "physical_behavior_certified": False}
    sock = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
    begun = False
    started = time.monotonic()
    with (output / "events.jsonl").open("w") as events, (output / "samples.jsonl").open("w") as samples:
        def event(**data):
            events.write(json.dumps({"t_s": time.monotonic() - started, **data}) + "\n")
            events.flush()

        def receive():
            data = sock.recv(8192)
            count, kind, body = parse(data)
            event(direction="rx", count=count, kind=kind, payload_hex=body.hex())
            return count, kind, body

        def send(kind, payload=b""):
            event(direction="tx", kind=kind, payload_hex=payload.hex())
            sock.send(frame(kind, payload))

        def ack():
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                _, kind, body = receive()
                if kind != 6:
                    continue
                if not body or body[0] != 0:
                    raise ValueError(f"CNDE rejected request: {body!r}")
                return body[1:].decode("ascii").rstrip("\x00")
            raise TimeoutError("No CNDE acknowledgment")

        try:
            sock.settimeout(1.0)
            sock.connect((config["ip"], config["cnde"]["port"]))
            begun = True
            send(3)
            ack()
            send(1, struct.pack("<H", config["cnde"]["cycle_ms"]) + ",".join(fields).encode("ascii"))
            response = ack()
            if response.split(",") != types:
                raise ValueError(f"CNDE negotiated types {response!r}; expected {types!r}")
            result["negotiated_types"] = types
            send(2)
            ack()
            previous = None
            stamps = []
            deadline = time.monotonic() + config["duration_s"]
            sock.settimeout(min(0.5, config["duration_s"]))
            while time.monotonic() < deadline:
                try:
                    count, kind, body = receive()
                except socket.timeout:
                    continue
                except ValueError as error:
                    result["invalid_packets"] += 1
                    event(error=str(error))
                    continue
                if kind == 6:
                    if not body or body[0] != 0:
                        raise ValueError(f"CNDE error during stream: {body!r}")
                    continue
                if kind != 4 or len(body) != struct.calcsize(fmt):
                    result["invalid_packets"] += 1
                    continue
                values = struct.unpack(fmt, body)
                if not all(math.isfinite(v) for v in values):
                    result["invalid_packets"] += 1
                    continue
                if previous is not None:
                    gap = (count - previous) % 256
                    if gap == 0:
                        result["duplicates"] += 1
                        continue
                    if gap > 128:
                        result["invalid_packets"] += 1
                        continue
                    result["estimated_lost_packets"] += gap - 1
                previous = count
                stamp = time.monotonic()
                stamps.append(stamp)
                decoded, index = {}, 0
                for field in fields:
                    n = 6 if FIELD_TYPES[field][0].endswith("_6") else 1
                    decoded[field] = list(values[index:index+n]) if n > 1 else values[index]
                    index += n
                samples.write(json.dumps({"t_s": stamp - started, "count": count, **decoded}) + "\n")
                samples.flush()
                result["packets"] += 1
            gaps = [b-a for a, b in zip(stamps, stamps[1:])]
            result["mean_period_ms"] = 1000 * sum(gaps) / len(gaps) if gaps else None
            result["max_period_ms"] = 1000 * max(gaps) if gaps else None
            result["status"] = "OBSERVED" if result["packets"] >= 3 and not result["invalid_packets"] else "INSUFFICIENT_OR_INVALID_FEEDBACK"
        except KeyboardInterrupt:
            result["status"] = "INTERRUPTED"
            result["error"] = "Interrupted; attempted CNDE stop acknowledgment"
        except (OSError, ValueError, TimeoutError) as error:
            result["error"] = str(error)
        finally:
            if begun:
                try:
                    sock.settimeout(1.0)
                    send(3)
                    ack()
                    result["cleanup_confirmed"] = True
                except (OSError, ValueError, TimeoutError) as error:
                    result["cleanup_error"] = str(error)
                    result["status"] = "CLEANUP_UNCONFIRMED"
            sock.close()
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result
