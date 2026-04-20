/*
 * Copyright 2026 Manifold Tech Ltd.(manifoldtech.cn)
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
 * @brief GGA data structure
 */
struct GGAData {
    double timestamp;   // Seconds of the day
    double lat;         // Latitude (degrees, positive=North)
    double lon;         // Longitude (degrees, positive=East)
    double alt;         // Altitude (meters)
    int quality;        // Fix quality (0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float)
    int num_satellites; // Number of satellites
    double hdop;        // Horizontal dilution of precision
    bool valid;         // Whether data is valid
};

/**
 * @brief NMEA GGA sentence parser
 *
 * Format: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 */
class GGAParser {
public:
    /**
     * @brief Parse GGA sentence
     * @param sentence NMEA sentence string
     * @return Parsed result
     */
    static GGAData parse(const std::string& sentence) {
        GGAData data;
        data.valid = false;

        // Check if this is a GGA sentence
        if (sentence.find("GGA") == std::string::npos) {
            return data;
        }

        // Remove checksum
        std::string clean_sentence = sentence;
        size_t star_pos = sentence.find('*');
        if (star_pos != std::string::npos) {
            clean_sentence = sentence.substr(0, star_pos);
        }

        // Split fields
        std::vector<std::string> fields;
        std::stringstream ss(clean_sentence);
        std::string field;
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        // GGA requires at least 15 fields
        if (fields.size() < 15) {
            return data;
        }

        try {
            // Parse fix quality (field 6)
            if (fields[6].empty()) {
                return data;
            }
            data.quality = std::stoi(fields[6]);
            if (data.quality < 1) {
                return data; // Invalid fix
            }

            // Parse time (field 1): hhmmss.ss
            if (!fields[1].empty()) {
                double time_raw = std::stod(fields[1]);
                int hh = static_cast<int>(time_raw / 10000);
                int mm = static_cast<int>((time_raw - hh * 10000) / 100);
                double ss_frac = time_raw - hh * 10000 - mm * 100;
                data.timestamp = hh * 3600.0 + mm * 60.0 + ss_frac;
            }

            // Parse latitude (field 2,3): ddmm.mmmm,N/S
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

            // Parse longitude (field 4,5): dddmm.mmmm,E/W
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

            // Parse number of satellites (field 7)
            if (!fields[7].empty()) {
                data.num_satellites = std::stoi(fields[7]);
            }

            // Parse HDOP (field 8)
            if (!fields[8].empty()) {
                data.hdop = std::stod(fields[8]);
            } else {
                data.hdop = 99.0; // Default high HDOP
            }

            // Parse altitude (field 9)
            if (!fields[9].empty()) {
                data.alt = std::stod(fields[9]);
            }

            // If geoid separation is available (field 11), consider adding it
            // data.alt += geoid_separation;

            data.valid = true;

        } catch (const std::exception&) {
            data.valid = false;
        }

        return data;
    }

    /**
     * @brief Check GGA data quality
     * @param data GGA data
     * @param min_quality Minimum quality requirement
     * @param max_hdop Maximum HDOP requirement
     * @return Whether quality check passed
     */
    static bool checkQuality(const GGAData& data, int min_quality = 1, double max_hdop = 5.0) {
        if (!data.valid) return false;
        if (data.quality < min_quality) return false;
        if (data.hdop > max_hdop) return false;
        return true;
    }

    /**
     * @brief Get fix quality description string
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
