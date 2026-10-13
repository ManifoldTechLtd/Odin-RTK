#!/usr/bin/env python3
#
# Copyright 2026 Manifold Tech Ltd.(manifoldtech.cn)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""
RTK Driver Example
Model: BG550
Output topic: /odin1/nmea_sentence
Current version only requires GGA data
"""

import argparse
import json
import threading
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from nmea_time import NmeaTimestampResolver, sentence_utc_ns
from nmea_msgs.msg import Sentence
from nmea_msgs.msg import Gpgga
from nmea_msgs.msg import Gpgsa
from nmea_msgs.msg import Gprmc

try:
    import serial
except ImportError as e:
    raise SystemExit(
        "Missing dependency 'pyserial'. Install it (e.g. pip install pyserial) and retry."
    ) from e


class SerialSentenceReader:
    def __init__(self, port: str, baudrate: int):
        self._port = port
        self._baudrate = baudrate
        self._ser = None
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread = None
        self._incomplete = ""
        self._on_error = None
        self._opened_once = False
        self._last_error_key = None
        self._last_error_time = 0.0
        self._error_throttle_sec = 2.0
        self._bytes_total = 0
        self._sentences_total = 0
        self._last_stats_time = 0.0
        self._stats_period_sec = 2.0

    def start(self, on_sentence, on_error=None):
        if self._thread is not None:
            return

        self._stop_evt.clear()
        self._on_error = on_error
        self._thread = threading.Thread(
            target=self._run, args=(on_sentence,), daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None

        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _open_serial(self):
        with self._lock:
            if self._ser is not None:
                return
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0.2,
            )
            if not self._opened_once:
                self._opened_once = True
                print(
                    f"Serial opened: {self._port}@{self._baudrate}",
                    flush=True,
                )

    def _run(self, on_sentence):
        while not self._stop_evt.is_set() and rclpy.ok():
            try:
                self._open_serial()
                with self._lock:
                    ser = self._ser

                if ser is None:
                    time.sleep(0.2)
                    continue

                chunk = ser.read(400)
                if not chunk:
                    continue

                text = chunk.decode("utf-8", errors="ignore")
                self._bytes_total += len(chunk)
                received = self._incomplete + text

                start = 0
                while True:
                    idx = received.find("\r\n", start)
                    if idx == -1:
                        break

                    sentence = received[start : idx + 2]
                    if sentence:
                        self._sentences_total += 1
                        on_sentence(sentence)

                    start = idx + 2

                self._incomplete = received[start:]

                now = time.time()
                if now - self._last_stats_time >= self._stats_period_sec:
                    self._last_stats_time = now
                    print(
                        f"Serial stats: bytes={self._bytes_total} sentences={self._sentences_total}",
                        flush=True,
                    )
            except Exception:
                err = traceback.format_exc()
                now = time.time()
                key = err.splitlines()[-1] if err else "unknown"
                if (
                    key != self._last_error_key
                    or (now - self._last_error_time) >= self._error_throttle_sec
                ):
                    self._last_error_key = key
                    self._last_error_time = now
                    print(err, flush=True)
                if self._on_error is not None:
                    try:
                        self._on_error(err)
                    except Exception:
                        pass
                with self._lock:
                    if self._ser is not None:
                        try:
                            self._ser.close()
                        except Exception:
                            pass
                        self._ser = None
                time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--topic", default="/odin1/nmea_sentence")
    parser.add_argument("--topic_gga", default="/odin1/gga")
    parser.add_argument("--topic_gsa", default="/odin1/gsa")
    parser.add_argument("--topic_rmc", default="/odin1/rmc")
    parser.add_argument("--node", default="nmea_sentence_publisher")
    parser.add_argument("--stamp-mode", choices=("receive", "utc"), default="receive")
    parser.add_argument("--utc-offset-sec", type=float, default=0.0)
    parser.add_argument("--max-sample-age-sec", type=float, default=2.0)
    parser.add_argument("--future-tolerance-sec", type=float, default=0.05)
    parser.add_argument("--time-diagnostics", choices=("true", "false"), default="false")
    args = parser.parse_args()
    try:
        resolver = NmeaTimestampResolver(args.stamp_mode, args.utc_offset_sec,
                                         args.max_sample_age_sec, args.future_tolerance_sec)
    except ValueError as exc:
        parser.error(str(exc))

    rclpy.init()

    node = Node(args.node)
    pub_raw = node.create_publisher(Sentence, args.topic, 50)
    pub_gga = node.create_publisher(Gpgga, args.topic_gga, 50)
    pub_gsa = node.create_publisher(Gpgsa, args.topic_gsa, 50)
    pub_rmc = node.create_publisher(Gprmc, args.topic_rmc, 50)
    pub_timing = node.create_publisher(String, "/odin1/nmea_timing", 50) if args.time_diagnostics == "true" else None
    reader = SerialSentenceReader(port=args.port, baudrate=args.baudrate)

    node.get_logger().info(f"Node started: {args.node}")
    node.get_logger().info(f"NMEA stamp mode={args.stamp_mode}, utc_offset_sec={args.utc_offset_sec}")
    node.get_logger().info(f"Publishing nmea_msgs/Sentence (raw) on: {args.topic}")
    node.get_logger().info(f"Publishing nmea_msgs/Gpgga on: {args.topic_gga}")
    node.get_logger().info(f"Publishing nmea_msgs/Gpgsa on: {args.topic_gsa}")
    node.get_logger().info(f"Publishing nmea_msgs/Gprmc on: {args.topic_rmc}")
    print(
        f"Starting publisher node={args.node} topic={args.topic} port={args.port}@{args.baudrate}",
        flush=True,
    )
    print("If you see 'Permission denied', fix serial permissions (dialout group).", flush=True)

    publish_count = {"n": 0}
    last_pub_print = {"t": time.time()}

    def _sentence_type(s: str):
        line = (s or "").strip()
        if not line:
            return None
        if line[0] not in ("$", "!"):
            return None
        payload = line[1:]
        comma = payload.find(",")
        if comma <= 0:
            return None
        header = payload[:comma]
        if len(header) < 3:
            return None
        return header[-3:].upper()

    def _nmea_fields(s: str):
        line = (s or "").strip()
        if not line:
            return None
        if line[0] not in ("$", "!"):
            return None
        if "*" in line:
            line = line.split("*", 1)[0]
        return line.split(",")

    def _safe_set(obj, name: str, value):
        if hasattr(obj, name):
            setattr(obj, name, value)

    def _parse_lat_lon(lat_raw: str, lat_dir: str, lon_raw: str, lon_dir: str):
        # NMEA: lat ddmm.mmmm, lon dddmm.mmmm
        def _parse_dm(dm: str, deg_len: int):
            if not dm:
                return None
            try:
                d = float(dm)
            except Exception:
                return None
            deg = int(d // 100)
            minutes = d - deg * 100
            return deg + minutes / 60.0

        lat = _parse_dm(lat_raw, 2)
        lon = _parse_dm(lon_raw, 3)
        if lat is not None and (lat_dir or "").upper() == "S":
            lat = -lat
        if lon is not None and (lon_dir or "").upper() == "W":
            lon = -lon
        return lat, lon

    def on_sentence(s: str):
        receive_ns = node.get_clock().now().nanoseconds
        stamp_ns = None
        error = ""
        try:
            if args.stamp_mode == "utc" and abs(receive_ns - time.time_ns()) > 100_000_000:
                raise ValueError("utc 模式要求 ROS 时钟与主机 UTC 同轴，不支持未对齐的 /clock")
            stamp_ns = resolver.resolve(s, receive_ns)
        except ValueError as exc:
            error = str(exc)
        if pub_timing is not None and _sentence_type(s) in ("GGA", "RMC"):
            utc_ns = None
            try:
                utc_ns = sentence_utc_ns(s, receive_ns - resolver.offset_ns)
            except ValueError as exc:
                error = error or str(exc)
            pub_timing.publish(String(data=json.dumps({
                "mode": args.stamp_mode, "receive_ns": receive_ns, "utc_ns": utc_ns,
                "stamp_ns": stamp_ns, "offset_ns": resolver.offset_ns,
                "accepted": stamp_ns is not None, "error": error,
            }, allow_nan=False)))
        if stamp_ns is None:
            node.get_logger().warning(f"Skip NMEA sample: {error}", throttle_duration_sec=2.0)
            return
        stamp = Time(nanoseconds=stamp_ns).to_msg()

        raw_msg = Sentence()
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = "antenna"
        raw_msg.sentence = s
        pub_raw.publish(raw_msg)

        stype = _sentence_type(s)
        fields = _nmea_fields(s)
        if not fields or len(fields) < 2:
            return

        if stype == "GGA":
            # $..GGA,time,lat,N,lon,E,qual,num_sats,hdop,alt,M,undulation,M,diff_age,station_id
            if len(fields) < 10:
                return
            msg = Gpgga()
            msg.header.stamp = stamp
            msg.header.frame_id = "antenna"
            try:
                _safe_set(msg, "utc_seconds", float(fields[1]) if fields[1] else 0.0)
            except Exception:
                pass
            try:
                lat_dir = fields[3] if len(fields) > 3 else ""
                lon_dir = fields[5] if len(fields) > 5 else ""
                lat_deg, lon_deg = _parse_lat_lon(fields[2] if len(fields) > 2 else "", lat_dir,
                                                  fields[4] if len(fields) > 4 else "", lon_dir)
                if lat_deg is not None:
                    _safe_set(msg, "lat", abs(float(lat_deg)))
                _safe_set(msg, "lat_dir", lat_dir)
                if lon_deg is not None:
                    _safe_set(msg, "lon", abs(float(lon_deg)))
                _safe_set(msg, "lon_dir", lon_dir)
            except Exception:
                pass
            try:
                v = int(fields[6]) if fields[6] else 0
                _safe_set(msg, "gps_qual", v if v >= 0 else 0)
            except Exception:
                pass
            try:
                v = int(fields[7]) if fields[7] else 0
                _safe_set(msg, "num_sats", v if v >= 0 else 0)
            except Exception:
                pass
            try:
                _safe_set(msg, "hdop", float(fields[8]) if fields[8] else 0.0)
            except Exception:
                pass
            try:
                _safe_set(msg, "alt", float(fields[9]) if fields[9] else 0.0)
            except Exception:
                pass
            if len(fields) > 11:
                try:
                    _safe_set(msg, "undulation", float(fields[11]) if fields[11] else 0.0)
                except Exception:
                    pass
            if len(fields) > 13:
                try:
                    # In ROS Noetic nmea_msgs/Gpgga.diff_age is an unsigned integer.
                    v = int(float(fields[13])) if fields[13] else 0
                    _safe_set(msg, "diff_age", v if v >= 0 else 0)
                except Exception:
                    pass
            if len(fields) > 14:
                _safe_set(msg, "station_id", fields[14])

            pub_gga.publish(msg)

        elif stype == "GSA":
            # $..GSA,mode1,mode2,sv1..sv12,pdop,hdop,vdop
            msg = Gpgsa()
            msg.header.stamp = stamp
            msg.header.frame_id = "antenna"

            if len(fields) > 1:
                _safe_set(msg, "auto_manual_mode", fields[1])
            if len(fields) > 2:
                try:
                    _safe_set(msg, "fix_mode", int(fields[2]) if fields[2] else 0)
                except Exception:
                    pass

            sv_ids = []
            for i in range(3, min(len(fields), 15)):
                if fields[i]:
                    try:
                        sv_ids.append(int(fields[i]))
                    except Exception:
                        pass
            _safe_set(msg, "sv_ids", sv_ids)

            if len(fields) > 15:
                try:
                    _safe_set(msg, "pdop", float(fields[15]) if fields[15] else 0.0)
                except Exception:
                    pass
            if len(fields) > 16:
                try:
                    _safe_set(msg, "hdop", float(fields[16]) if fields[16] else 0.0)
                except Exception:
                    pass
            if len(fields) > 17:
                try:
                    _safe_set(msg, "vdop", float(fields[17]) if fields[17] else 0.0)
                except Exception:
                    pass

            pub_gsa.publish(msg)

        elif stype == "RMC":
            # $..RMC,time,status,lat,N,lon,E,sog,cog,date,magvar,magvar_dir,mode
            if len(fields) < 10:
                return
            msg = Gprmc()
            msg.header.stamp = stamp
            msg.header.frame_id = "antenna"

            try:
                _safe_set(msg, "utc_seconds", float(fields[1]) if fields[1] else 0.0)
            except Exception:
                pass
            if len(fields) > 2:
                _safe_set(msg, "status", fields[2])
            if len(fields) > 6:
                try:
                    lat_dir = fields[4] if len(fields) > 4 else ""
                    lon_dir = fields[6] if len(fields) > 6 else ""
                    lat_deg, lon_deg = _parse_lat_lon(fields[3] if len(fields) > 3 else "", lat_dir,
                                                      fields[5] if len(fields) > 5 else "", lon_dir)
                    if lat_deg is not None:
                        _safe_set(msg, "lat", abs(float(lat_deg)))
                    _safe_set(msg, "lat_dir", lat_dir)
                    if lon_deg is not None:
                        _safe_set(msg, "lon", abs(float(lon_deg)))
                    _safe_set(msg, "lon_dir", lon_dir)
                except Exception:
                    pass
            if len(fields) > 7:
                try:
                    _safe_set(msg, "speed", float(fields[7]) if fields[7] else 0.0)
                except Exception:
                    pass
            if len(fields) > 8:
                try:
                    _safe_set(msg, "track", float(fields[8]) if fields[8] else 0.0)
                except Exception:
                    pass
            if len(fields) > 9:
                _safe_set(msg, "date", fields[9])
            if len(fields) > 10:
                try:
                    _safe_set(msg, "mag_var", float(fields[10]) if fields[10] else 0.0)
                except Exception:
                    pass
            if len(fields) > 11:
                _safe_set(msg, "mag_var_direction", fields[11])
            if len(fields) > 12:
                _safe_set(msg, "mode_indicator", fields[12])

            pub_rmc.publish(msg)

        publish_count["n"] += 1

        now = time.time()
        if now - last_pub_print["t"] >= 2.0:
            last_pub_print["t"] = now
            print(f"Published: {publish_count['n']} messages", flush=True)

    def on_error(err: str):
        node.get_logger().error(
            "Serial error (see terminal): %s"
            % (err.splitlines()[-1] if err else "unknown")
        )

    reader.start(on_sentence, on_error)

    try:
        rclpy.spin(node)
    finally:
        reader.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
