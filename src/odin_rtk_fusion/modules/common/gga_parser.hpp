/*
 * Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <cmath>

namespace odin_rtk_fusion {

/**
 * @brief GGA数据结构
 */
struct GGAData {
    double timestamp;   // 当天秒数
    double lat;         // 纬度（度，正为北纬）
    double lon;         // 经度（度，正为东经）
    double alt;         // 海拔高度（米）
    int quality;        // 定位质量 (0=无效, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float)
    int num_satellites; // 卫星数量
    double hdop;        // 水平精度因子
    bool valid;         // 数据是否有效
};

/**
 * @brief NMEA GGA语句解析器
 *
 * 格式: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 */
class GGAParser {
public:
    /**
     * @brief 解析GGA语句
     * @param sentence NMEA语句字符串
     * @return 解析结果
     */
    static GGAData parse(const std::string& sentence) {
        GGAData data;
        data.valid = false;

        // 检查是否为GGA语句
        if (sentence.find("GGA") == std::string::npos) {
            return data;
        }

        // 去除校验和部分
        std::string clean_sentence = sentence;
        size_t star_pos = sentence.find('*');
        if (star_pos != std::string::npos) {
            clean_sentence = sentence.substr(0, star_pos);
        }

        // 分割字段
        std::vector<std::string> fields;
        std::stringstream ss(clean_sentence);
        std::string field;
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        // GGA至少需要15个字段
        if (fields.size() < 15) {
            return data;
        }

        try {
            // 解析定位质量 (字段6)
            if (fields[6].empty()) {
                return data;
            }
            data.quality = std::stoi(fields[6]);
            if (data.quality < 1) {
                return data; // 无效定位
            }

            // 解析时间 (字段1): hhmmss.ss
            if (!fields[1].empty()) {
                double time_raw = std::stod(fields[1]);
                int hh = static_cast<int>(time_raw / 10000);
                int mm = static_cast<int>((time_raw - hh * 10000) / 100);
                double ss_frac = time_raw - hh * 10000 - mm * 100;
                data.timestamp = hh * 3600.0 + mm * 60.0 + ss_frac;
            }

            // 解析纬度 (字段2,3): ddmm.mmmm,N/S
            if (!fields[2].empty() && !fields[3].empty()) {
                double lat_raw = std::stod(fields[2]);
                int lat_deg = static_cast<int>(lat_raw / 100);
                double lat_min = lat_raw - lat_deg * 100;
                data.lat = lat_deg + lat_min / 60.0;
                if (fields[3] == "S" || fields[3] == "s") {
                    data.lat = -data.lat;
                }
            } else {
                return data;
            }

            // 解析经度 (字段4,5): dddmm.mmmm,E/W
            if (!fields[4].empty() && !fields[5].empty()) {
                double lon_raw = std::stod(fields[4]);
                int lon_deg = static_cast<int>(lon_raw / 100);
                double lon_min = lon_raw - lon_deg * 100;
                data.lon = lon_deg + lon_min / 60.0;
                if (fields[5] == "W" || fields[5] == "w") {
                    data.lon = -data.lon;
                }
            } else {
                return data;
            }

            // 解析卫星数量 (字段7)
            if (!fields[7].empty()) {
                data.num_satellites = std::stoi(fields[7]);
            }

            // 解析HDOP (字段8)
            if (!fields[8].empty()) {
                data.hdop = std::stod(fields[8]);
            } else {
                data.hdop = 99.0; // 默认高HDOP
            }

            // 解析海拔高度 (字段9)
            if (!fields[9].empty()) {
                data.alt = std::stod(fields[9]);
            }

            // 如果有大地水准面高度差 (字段11)，可以考虑加上
            // data.alt += geoid_separation;

            data.valid = true;

        } catch (const std::exception&) {
            data.valid = false;
        }

        return data;
    }

    /**
     * @brief 检查GGA数据质量
     * @param data GGA数据
     * @param min_quality 最小质量要求
     * @param max_hdop 最大HDOP要求
     * @return 是否通过质量检查
     */
    static bool checkQuality(const GGAData& data, int min_quality = 1, double max_hdop = 5.0) {
        if (!data.valid) return false;
        if (data.quality < min_quality) return false;
        if (data.hdop > max_hdop) return false;
        return true;
    }

    /**
     * @brief 获取定位质量描述
     */
    static std::string qualityString(int quality) {
        switch (quality) {
            case 0: return "Invalid";
            case 1: return "GPS";
            case 2: return "DGPS";
            case 3: return "PPS";
            case 4: return "RTK Fix";
            case 5: return "RTK Float";
            case 6: return "Estimated";
            case 7: return "Manual";
            case 8: return "Simulation";
            default: return "Unknown";
        }
    }
};

} // namespace odin_rtk_fusion
