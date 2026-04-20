# ODINBIN Binary Format Specification

## Overview

ODINBIN is a simple binary format for storing odometry and GNSS data, designed to be read without any ROS dependencies.

## File Structure

```
[File Header]
[Record 0]
[Record 1]
...
[Record N-1]
```

All records are sorted by timestamp.

## File Header (20 bytes)

| Offset | Size | Type   | Description                    |
|--------|------|--------|--------------------------------|
| 0      | 8    | char[] | Magic: "ODINBIN\0"            |
| 8      | 4    | uint32 | Version (currently 1)          |
| 12     | 8    | uint64 | Number of records              |

## Record Format

Each record starts with:

| Offset | Size | Type   | Description                              |
|--------|------|--------|------------------------------------------|
| 0      | 1    | uint8  | Message type (0=ODOM, 1=BESTPOS, 2=HEADING2) |
| 1      | 8    | float64| Timestamp (seconds, Unix epoch)          |
| 9      | var  | bytes  | Payload (size depends on type)           |

### Type 0: ODOM (payload = 56 bytes)

| Offset | Size | Type    | Field |
|--------|------|---------|-------|
| 0      | 8    | float64 | px    |
| 8      | 8    | float64 | py    |
| 16     | 8    | float64 | pz    |
| 24     | 8    | float64 | qw    |
| 32     | 8    | float64 | qx    |
| 40     | 8    | float64 | qy    |
| 48     | 8    | float64 | qz    |

### Type 1: BESTPOS (payload = 57 bytes)

| Offset | Size | Type    | Field        |
|--------|------|---------|--------------|
| 0      | 8    | float64 | lat (deg)    |
| 8      | 8    | float64 | lon (deg)    |
| 16     | 8    | float64 | hgt (m, MSL)|
| 24     | 4    | float32 | undulation   |
| 28     | 4    | float32 | lat_stdev    |
| 32     | 4    | float32 | lon_stdev    |
| 36     | 4    | float32 | hgt_stdev    |
| 40     | 4    | uint32  | sol_status   |
| 44     | 4    | uint32  | pos_type     |
| 48     | 1    | uint8   | num_sol_svs  |
| 49     | 4    | float32 | diff_age     |
| 53     | 4    | float32 | sol_age      |

**sol_status**: 0=SOL_COMPUTED, others=invalid
**pos_type**: 48=L1_INT(RTK Fix), 50=NARROW_INT(RTK Fix), 32=L1_FLOAT, 16=SINGLE, etc.

### Type 2: HEADING2 (payload = 28 bytes)

| Offset | Size | Type    | Field         |
|--------|------|---------|---------------|
| 0      | 4    | float32 | heading (deg) |
| 4      | 4    | float32 | pitch (deg)   |
| 8      | 4    | float32 | heading_stdev |
| 12     | 4    | float32 | pitch_stdev   |
| 16     | 4    | float32 | length (m)    |
| 20     | 4    | uint32  | sol_status    |
| 24     | 4    | uint32  | pos_type      |

## C++ Reading Example

```cpp
#include <fstream>
#include <cstdint>
#include <cstring>

struct OdinBinHeader {
    char magic[8];
    uint32_t version;
    uint64_t num_records;
};

struct OdomPayload {
    double px, py, pz;
    double qw, qx, qy, qz;
};

struct BestposPayload {
    double lat, lon, hgt;
    float undulation;
    float lat_stdev, lon_stdev, hgt_stdev;
    uint32_t sol_status, pos_type;
    uint8_t num_sol_svs;
    float diff_age, sol_age;
};

struct Heading2Payload {
    float heading, pitch;
    float heading_stdev, pitch_stdev;
    float length;
    uint32_t sol_status, pos_type;
};

// Read file
std::ifstream in("data.bin", std::ios::binary);
OdinBinHeader header;
in.read(reinterpret_cast<char*>(&header), sizeof(header));

for (uint64_t i = 0; i < header.num_records; i++) {
    uint8_t type;
    double timestamp;
    in.read(reinterpret_cast<char*>(&type), 1);
    in.read(reinterpret_cast<char*>(&timestamp), 8);

    if (type == 0) {
        OdomPayload odom;
        in.read(reinterpret_cast<char*>(&odom), sizeof(odom));
        // use odom...
    } else if (type == 1) {
        BestposPayload bp;
        in.read(reinterpret_cast<char*>(&bp), sizeof(bp));
        // use bestpos...
    } else if (type == 2) {
        Heading2Payload h2;
        in.read(reinterpret_cast<char*>(&h2), sizeof(h2));
        // use heading2...
    }
}
```

**Note**: `BestposPayload` struct has packing issues due to the `uint8_t` field.
Read fields individually with `memcpy` for safety, or use `#pragma pack(push, 1)`.
