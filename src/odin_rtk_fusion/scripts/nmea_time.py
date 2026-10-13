import calendar
from datetime import datetime, timezone
from decimal import Decimal
import math
import re


NS_PER_SEC = 1_000_000_000
NS_PER_DAY = 86_400 * NS_PER_SEC


def seconds_to_ns(value):
    if not math.isfinite(float(value)):
        raise ValueError("时间参数必须是有限值")
    return int(Decimal(str(value)) * NS_PER_SEC)


def utc_time_of_day_ns(value):
    match = re.fullmatch(r"(\d{2})(\d{2})(\d{2})(?:\.(\d{1,9}))?", value, flags=re.ASCII)
    if match is None:
        raise ValueError("NMEA UTC 必须为 hhmmss[.fffffffff]")
    hour, minute, second = (int(match[index]) for index in (1, 2, 3))
    if hour > 23 or minute > 59 or second > 59:
        raise ValueError("NMEA UTC 越界（闰秒需单独处理，不自动修正）")
    fraction = int((match[4] or "").ljust(9, "0"))
    return (hour * 3600 + minute * 60 + second) * NS_PER_SEC + fraction


def sentence_utc_ns(sentence, reference_ns):
    line = sentence.strip()
    if not line.startswith("$"):
        raise ValueError("不是有效 NMEA 语句")
    payload, separator, checksum_text = line[1:].partition("*")
    fields = payload.split(",")
    kind = fields[0][-3:].upper()
    if kind not in ("GGA", "RMC"):
        return None
    if separator:
        if not re.fullmatch(r"[0-9a-fA-F]{2}", checksum_text):
            raise ValueError("NMEA 校验和格式无效")
        checksum = 0
        for character in payload:
            checksum ^= ord(character)
        if checksum != int(checksum_text, 16):
            raise ValueError("NMEA 校验和不匹配")
    if len(fields) < 2:
        raise ValueError("NMEA 缺 UTC 字段")
    time_ns = utc_time_of_day_ns(fields[1])
    if kind == "RMC":
        if len(fields) < 10 or not re.fullmatch(r"\d{6}", fields[9], flags=re.ASCII):
            raise ValueError("RMC 缺有效 ddmmyy 日期")
        day, month, year = int(fields[9][:2]), int(fields[9][2:4]), int(fields[9][4:])
        reference_year = datetime.fromtimestamp(reference_ns // NS_PER_SEC, timezone.utc).year
        century = reference_year // 100 * 100
        dates = []
        for base in (century - 100, century, century + 100):
            try:
                dates.append(calendar.timegm(datetime(base + year, month, day).timetuple()) * NS_PER_SEC + time_ns)
            except ValueError:
                pass
        if not dates:
            raise ValueError("RMC 日期越界")
        return min(dates, key=lambda value: abs(value - reference_ns))
    midnight = reference_ns // NS_PER_DAY * NS_PER_DAY
    return min((midnight + time_ns + delta for delta in (-NS_PER_DAY, 0, NS_PER_DAY)),
               key=lambda value: abs(value - reference_ns))


class NmeaTimestampResolver:
    def __init__(self, mode="receive", offset_sec=0.0, max_age_sec=2.0, future_tolerance_sec=0.05):
        if mode not in ("receive", "utc"):
            raise ValueError("时间模式只能是 receive 或 utc")
        self.mode = mode
        self.offset_ns = seconds_to_ns(offset_sec)
        self.max_age_ns = seconds_to_ns(max_age_sec)
        self.future_tolerance_ns = seconds_to_ns(future_tolerance_sec)
        if self.max_age_ns <= 0 or self.future_tolerance_ns < 0:
            raise ValueError("最大年龄须大于 0、未来容差须非负")
        if mode == "receive" and self.offset_ns:
            raise ValueError("时间偏置只用于 utc 模式")
        self.last_stamp = {}

    def resolve(self, sentence, receive_ns):
        if self.mode == "receive":
            return receive_ns
        utc_ns = sentence_utc_ns(sentence, receive_ns - self.offset_ns)
        if utc_ns is None:
            return receive_ns
        stamp_ns = utc_ns + self.offset_ns
        age_ns = receive_ns - stamp_ns
        if age_ns > self.max_age_ns or age_ns < -self.future_tolerance_ns:
            raise ValueError(f"NMEA 采样时间过期或时间轴不一致：age={age_ns / NS_PER_SEC:.6f}s")
        kind = sentence.strip().split(",", 1)[0][-3:].upper()
        if stamp_ns <= self.last_stamp.get(kind, -1):
            raise ValueError(f"{kind} 采样时间重复或倒退")
        self.last_stamp[kind] = stamp_ns
        return stamp_ns
