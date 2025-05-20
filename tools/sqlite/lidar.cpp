/******************************************************************************
 * @brief LiDAR Class for reading and parsing LAS 1.4 files
 *
 * @file lidar.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-20
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <regex>
#include <sqlite3.h>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

/******************************************************************************
 * @brief LiDAR Definition for LAS 1.4
 *
 * @note Contains structures and functions to read and parse LAS 1.4 files.
 *       But only the fields we need for our application so some of the fields
 *       typically found in a LAS header or point data record are not included.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-21
 ******************************************************************************/
class LiDARVersion1_4
{
    public:
        ////////////////////////////////////////////
        // Enumerations for LAS 1.4
        ////////////////////////////////////////////
        enum class PointClassification : uint8_t
        {
            CreatedNeverClassified = 0,     // Created, never classified
            Unclassified           = 1,     // Unclassified
            Ground                 = 2,     // Ground
            LowVegetation          = 3,     // Low Vegetation
            MediumVegetation       = 4,     // Medium Vegetation
            HighVegetation         = 5,     // High Vegetation
            Building               = 6,     // Building
            LowPointNoise          = 7,     // Low Point (Noise)
            Reserved8              = 8,     // Reserved
            Water                  = 9,     // Water
            Rail                   = 10,    // Rail
            RoadSurface            = 11,    // Road Surface
            Reserved12             = 12,    // Reserved
            WireGuardShield        = 13,    // Wire – Guard (Shield)
            WireConductorPhase     = 14,    // Wire – Conductor (Phase)
            TransmissionTower      = 15,    // Transmission Tower
            WireStructureConnector = 16,    // Wire-Structure Connector
            BridgeDeck             = 17,    // Bridge Deck
            HighNoise              = 18,    // High Noise
            OverheadStructure      = 19,    // Overhead Structure
            IgnoredGround          = 20,    // Ignored Ground
            Snow                   = 21,    // Snow
            TemporalExclusion      = 22,    // Temporal Exclusion
            Reserved               = 23,    // 23–63
            UserDefinable          = 64     // 64–255
        };

    private:
        ////////////////////////////////////////////
        // Constants for LAS 1.4
        ////////////////////////////////////////////

        // Offsets for the LAS 1.4 header (Table 3)
        static constexpr size_t OFF_headerSize            = 94;     // Header Size                       | unsigned short     | 2 bytes
        static constexpr size_t OFF_offsetToPointData     = 96;     // Offset to Point Data              | unsigned long      | 4 bytes
        static constexpr size_t OFF_numVLRs               = 100;    // Number of Variable Length Records | unsigned long      | 4 bytes
        static constexpr size_t OFF_fmt                   = 104;    // Point Data Record Format          | unsigned char      | 1 byte
        static constexpr size_t OFF_pointDataRecordLength = 105;    // Point Data Record Length          | unsigned short     | 2 bytes
        static constexpr size_t OFF_xSF                   = 131;    // X Scale Factor                    | double             | 8 bytes
        static constexpr size_t OFF_ySF                   = 139;    // Y Scale Factor                    | double             | 8 bytes
        static constexpr size_t OFF_zSF                   = 147;    // Z Scale Factor                    | double             | 8 bytes
        static constexpr size_t OFF_xOF                   = 155;    // X Offset                          | double             | 8 bytes
        static constexpr size_t OFF_yOF                   = 163;    // Y Offset                          | double             | 8 bytes
        static constexpr size_t OFF_zOF                   = 171;    // Z Offset                          | double             | 8 bytes
        static constexpr size_t OFF_MX                    = 179;    // X Max                             | double             | 8 bytes
        static constexpr size_t OFF_MY                    = 195;    // Y Max                             | double             | 8 bytes
        static constexpr size_t OFF_MZ                    = 211;    // Z Max                             | double             | 8 bytes
        static constexpr size_t OFF_mx                    = 187;    // X Min                             | double             | 8 bytes
        static constexpr size_t OFF_my                    = 203;    // Y Min                             | double             | 8 bytes
        static constexpr size_t OFF_mz                    = 219;    // Z Min                             | double             | 8 bytes
        static constexpr size_t OFF_numPts                = 247;    // Number of Point Records           | unsigned long long | 8 bytes

        // Path to the SQLite database
        static constexpr const char* DB_PATH = "../../data/LiDAR/sqlite/MDRS.db";

        ////////////////////////////////////////////
        // Structures for LAS 1.4
        ////////////////////////////////////////////

        /******************************************************************************
         * @brief Minimal LAS header: only the fields we need
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        struct MinimalLASHeader
        {
                uint8_t pointDataFormat;
                double xScale, yScale, zScale;
                double xOffset, yOffset, zOffset;
                double maxX, maxY, maxZ;
                double minX, minY, minZ;
                uint16_t headerSize;
                uint64_t numberOfPointRecords;
                uint32_t numVLRs;
                uint32_t offsetToPointData;
                uint16_t pointDataRecordLength;
        };

        /******************************************************************************
         * @brief VLR header
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
#pragma pack(push, 1)

        struct VLRHeader
        {
                uint16_t reserved;                   // always 0
                char userID[16];                     // e.g. "LASF_Projection"
                uint16_t recordID;                   // vendor code
                uint16_t recordLengthAfterHeader;    // payload length
                char description[32];                // human text
        };

#pragma pack(pop)

        /******************************************************************************
         * @brief Point data record (Format 6)
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        struct Point6
        {
                int32_t X, Y, Z;    // raw ints
                uint8_t classification;
        };

        /******************************************************************************
         * @brief Data structure for a point row to be inserted into SQLite
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        struct PointRow
        {
                std::string id;
                uint64_t pointId;
                double easting;
                double northing;
                double altitude;
                std::pair<int, char> zone;
                std::string classification;
        };

        ////////////////////////////////////////////
        // Helper Functions for LAS 1.4
        ////////////////////////////////////////////

        /******************************************************************************
         * @brief Converts a uint8_t value to a PointClassification enum
         *
         * @param v - The uint8_t value to convert
         * @return PointClassification - The corresponding PointClassification enum value
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        static PointClassification makeClassification(uint8_t v)
        {
            // 0-22: Defined by ASPRS
            if (v <= 22)
            {
                return static_cast<PointClassification>(v);
            }

            // 23-63: Reserved
            if (v >= 23 && v <= 63)
            {
                return PointClassification::Reserved;
            }

            // 64-255: User definable
            return PointClassification::UserDefinable;
        }

        /******************************************************************************
         * @brief Reads a little-endian 32-bit integer from a buffer
         *
         * @param buf - The buffer to read from
         * @param off - The offset to read from
         * @return int32_t - The little-endian 32-bit integer value
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        static int32_t readLE32(const char* buf, size_t off)
        {
            auto b0 = static_cast<uint8_t>(buf[off + 0]);
            auto b1 = static_cast<uint8_t>(buf[off + 1]);
            auto b2 = static_cast<uint8_t>(buf[off + 2]);
            auto b3 = static_cast<uint8_t>(buf[off + 3]);
            return int32_t(b0 | (b1 << 8) | (b2 << 16) | (b3 << 24));
        }

        /******************************************************************************
         * @brief Trims a string to remove trailing null characters and spaces
         *
         * @param buf - The buffer to trim
         * @param len - The length of the buffer
         * @return std::string - The trimmed string
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        static std::string trim(const char* buf, size_t len)
        {
            size_t e = len;
            while (e > 0 && (buf[e - 1] == '\0' || buf[e - 1] == ' '))
                --e;
            return std::string(buf, buf + e);
        }

        ////////////////////////////////////////////
        // Functional Methods for LAS 1.4
        ////////////////////////////////////////////

        /******************************************************************************
         * @brief Reads the minimal LAS header from a file
         *
         * @param in - The input file stream
         * @return MinimalLASHeader - The minimal LAS header
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        MinimalLASHeader readMinimalHeader(std::ifstream& in)
        {
            // Create a MinimalLASHeader object
            MinimalLASHeader h{};

            // Read the size of the header
            in.seekg(OFF_headerSize, std::ios::beg);
            in.read(reinterpret_cast<char*>(&h.headerSize), sizeof(h.headerSize));

            // Read the number of variable length records
            in.seekg(OFF_numVLRs, std::ios::beg);
            in.read(reinterpret_cast<char*>(&h.numVLRs), sizeof(h.numVLRs));

            // Read the entire header into a buffer
            std::vector<char> buf(h.headerSize);
            in.seekg(0, std::ios::beg);
            in.read(buf.data(), buf.size());

            // Use memcpy to extract the fields from the buffer
            std::memcpy(&h.pointDataFormat, buf.data() + OFF_fmt, sizeof(h.pointDataFormat));
            std::memcpy(&h.xScale, buf.data() + OFF_xSF, sizeof(h.xScale));
            std::memcpy(&h.yScale, buf.data() + OFF_ySF, sizeof(h.yScale));
            std::memcpy(&h.zScale, buf.data() + OFF_zSF, sizeof(h.zScale));
            std::memcpy(&h.xOffset, buf.data() + OFF_xOF, sizeof(h.xOffset));
            std::memcpy(&h.yOffset, buf.data() + OFF_yOF, sizeof(h.yOffset));
            std::memcpy(&h.zOffset, buf.data() + OFF_zOF, sizeof(h.zOffset));
            std::memcpy(&h.maxX, buf.data() + OFF_MX, sizeof(h.maxX));
            std::memcpy(&h.maxY, buf.data() + OFF_MY, sizeof(h.maxY));
            std::memcpy(&h.maxZ, buf.data() + OFF_MZ, sizeof(h.maxZ));
            std::memcpy(&h.minX, buf.data() + OFF_mx, sizeof(h.minX));
            std::memcpy(&h.minY, buf.data() + OFF_my, sizeof(h.minY));
            std::memcpy(&h.minZ, buf.data() + OFF_mz, sizeof(h.minZ));
            std::memcpy(&h.numberOfPointRecords, buf.data() + OFF_numPts, sizeof(h.numberOfPointRecords));
            std::memcpy(&h.offsetToPointData, buf.data() + OFF_offsetToPointData, sizeof(h.offsetToPointData));
            std::memcpy(&h.pointDataRecordLength, buf.data() + OFF_pointDataRecordLength, sizeof(h.pointDataRecordLength));

            // Return the simplified header
            return h;
        }

        /******************************************************************************
         * @brief Extracts the UTM zone from the first VLR
         *
         * @param in - The input file stream
         * @param h - The minimal LAS header
         * @return std::pair<int, char> - The UTM zone and hemisphere
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        std::pair<int, char> extractUTMZoneFromVLR1(std::ifstream& in, const MinimalLASHeader& h)
        {
            // 1) Seek to the first VLR, which lives at byte offset = headerSize
            in.seekg(h.headerSize, std::ios::beg);

            // 2) Read the 54-byte VLR header
            VLRHeader vh;
            in.read(reinterpret_cast<char*>(&vh), sizeof(vh));
            if (!in)
                throw std::runtime_error("Failed to read VLR #1 header");

            // 3) Read exactly recordLengthAfterHeader bytes of payload
            std::vector<char> payload(vh.recordLengthAfterHeader);
            in.read(payload.data(), payload.size());
            if (!in && vh.recordLengthAfterHeader > 0)
                throw std::runtime_error("Failed to read VLR #1 payload");

            // 4) Build a string from the payload
            std::string text(payload.begin(), payload.end());

            // 5) Use regex to find “UTM zone <digits><optional N/S>”
            static const std::regex re(R"(UTM\s+zone\s*([0-9]+)([NS])?)", std::regex::icase);
            std::smatch m;
            if (std::regex_search(text, m, re))
            {
                int zone  = std::stoi(m[1].str());
                char hemi = (m.size() >= 3 && !m[2].str().empty()) ? m[2].str()[0] : '?';
                return {zone, hemi};
            }
            return {-1, '?'};
        }

    public:
        /******************************************************************************
         * @brief Construct a new LiDARVersion1_4 object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        LiDARVersion1_4() = default;

        /******************************************************************************
         * @brief Destroy the LiDARVersion1_4 object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        ~LiDARVersion1_4() = default;

        ////////////////////////////////////////////
        // Console Printing Methods for LAS 1.4
        ////////////////////////////////////////////

        /******************************************************************************
         * @brief Converts a PointClassification enum to a string
         *
         * @param cls - The PointClassification enum value to convert
         * @return std::string - The corresponding string representation of the enum value
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        static std::string to_string(PointClassification cls)
        {
            switch (cls)
            {
                case PointClassification::CreatedNeverClassified: return "Created (Never Classified)";
                case PointClassification::Unclassified: return "Unclassified";
                case PointClassification::Ground: return "Ground";
                case PointClassification::LowVegetation: return "Low Vegetation";
                case PointClassification::MediumVegetation: return "Medium Vegetation";
                case PointClassification::HighVegetation: return "High Vegetation";
                case PointClassification::Building: return "Building";
                case PointClassification::LowPointNoise: return "Low Point (Noise)";
                case PointClassification::Reserved8: return "Reserved";
                case PointClassification::Water: return "Water";
                case PointClassification::Rail: return "Rail";
                case PointClassification::RoadSurface: return "Road Surface";
                case PointClassification::Reserved12: return "Reserved";
                case PointClassification::WireGuardShield: return "Wire – Guard (Shield)";
                case PointClassification::WireConductorPhase: return "Wire – Conductor (Phase)";
                case PointClassification::TransmissionTower: return "Transmission Tower";
                case PointClassification::WireStructureConnector: return "Wire-Structure Connector";
                case PointClassification::BridgeDeck: return "Bridge Deck";
                case PointClassification::HighNoise: return "High Noise";
                case PointClassification::OverheadStructure: return "Overhead Structure";
                case PointClassification::IgnoredGround: return "Ignored Ground";
                case PointClassification::Snow: return "Snow";
                case PointClassification::TemporalExclusion: return "Temporal Exclusion";
                case PointClassification::Reserved: return "Reserved";
                case PointClassification::UserDefinable: return "User Definable";
            }
            return "Unknown";
        }

        /******************************************************************************
         * @brief Prints the payload of a VLR as text+hex to the console
         *
         * @param p - The payload vector to print
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        static void dumpPayload(const std::vector<char>& p)
        {
            std::cout << "    Payload (" << p.size() << " bytes):\n      ";
            for (char c : p)
            {
                if (std::isprint((unsigned char) c))
                    std::cout << c;
                else
                    std::cout << "\\x" << std::hex << std::setw(2) << std::setfill('0') << (static_cast<int>(static_cast<unsigned char>(c))) << std::dec;
            }
            std::cout << "\n";
        }

        /******************************************************************************
         * @brief Decodes the projection VLR and prints the WKT to the console
         *
         * @param p - The payload vector to decode
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        static void decodeProjectionVLR(const std::vector<char>& p)
        {
            std::string s(p.begin(), p.end());
            auto pos = s.find("COMPD_CS[");
            if (pos != std::string::npos)
            {
                auto end = s.find('\0', pos);
                std::cout << "      WKT: " << s.substr(pos, end - pos) << "\n";
            }
        }

        /******************************************************************************
         * @brief Reads and scales the points from the LAS file then prints them to the console
         *
         * @param in - The input file stream
         * @param h - The minimal LAS header
         * @param numPoints - The number of points to read and scale (default is 5)
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        static void readAndScalePoints(std::ifstream& in, const MinimalLASHeader& h, uint64_t numPoints = 5)
        {
            // 1) Seek to first point
            in.seekg(h.offsetToPointData, std::ios::beg);

            std::vector<char> buf(h.pointDataRecordLength);

            if (numPoints > h.numberOfPointRecords)
            {
                numPoints = h.numberOfPointRecords;
            }
            else if (numPoints < 0)
            {
                numPoints = 0;
            }

            std::cout << "\nScaled Points (first 5):\n";
            for (uint64_t i = 0; i < numPoints && i < h.numberOfPointRecords; ++i)
            {
                in.read(buf.data(), buf.size());
                if (!in)
                    break;

                // extract raw ints at offsets 0,4,8
                int32_t rx = readLE32(buf.data(), 0);
                int32_t ry = readLE32(buf.data(), 4);
                int32_t rz = readLE32(buf.data(), 8);

                // classification is a single byte at offset 15
                uint8_t cls = static_cast<uint8_t>(buf[16]);

                double x    = rx * h.xScale + h.xOffset;
                double y    = ry * h.yScale + h.yOffset;
                double z    = rz * h.zScale + h.zOffset;

                std::cout << "  Point[" << i << "] = (" << x << ", " << y << ", " << z << ")"
                          << "  cls=" << int(cls) << "\n";
            }
        }

        /******************************************************************************
         * @brief Collects all points into a vector of PointRow to be inserted into SQLite
         *
         * @param in - The input file stream
         * @param h - The minimal LAS header
         * @param utmZone - The UTM zone for all points (e.g. 12, 'N')
         * @return std::vector<PointRow> - The vector of PointRow objects
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-21
         ******************************************************************************/
        static std::vector<PointRow> collectPointRecords(std::ifstream& in, const MinimalLASHeader& h, std::pair<int, char> utmZone, unsigned long long idOffset)
        {
            // Seek to point block
            in.seekg(h.offsetToPointData, std::ios::beg);

            std::vector<char> buf(h.pointDataRecordLength);
            std::vector<PointRow> rows;
            rows.reserve(h.numberOfPointRecords);

            for (uint64_t i = 0; i < h.numberOfPointRecords; ++i)
            {
                in.read(buf.data(), buf.size());
                if (!in)
                    break;

                // raw ints
                int32_t rx = readLE32(buf.data(), 0);
                int32_t ry = readLE32(buf.data(), 4);
                int32_t rz = readLE32(buf.data(), 8);

                // classification at offset 16
                uint8_t cls                 = static_cast<uint8_t>(buf[16]);
                PointClassification clsEnum = makeClassification(cls);
                std::string szCLS           = to_string(clsEnum);

                // scale & offset
                double easting  = rx * h.xScale + h.xOffset;
                double northing = ry * h.yScale + h.yOffset;
                double altitude = rz * h.zScale + h.zOffset;

                // name the point
                std::string id = std::to_string(easting) + "_" + std::to_string(northing);

                // add to the vector making the neighbor list empty
                rows.push_back({id, i + idOffset, easting, northing, altitude, utmZone, szCLS});
            }
            return rows;
        }

        /******************************************************************************
         * @brief Populates the SQLite database with the point data
         *
         * @param rows - The vector of PointRow objects to insert
         * @param neighborRadius - The radius for finding neighbors (default is 5.0)
         * @param dbPath - The path to the SQLite database (default is DB_PATH)
         * @return int - Returns 0 on success, -1 on failure
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-05-20
         ******************************************************************************/
        int PopulateSQL(const std::vector<PointRow>& rows, double neighborRadius = 5.0, const std::string& dbPath = DB_PATH)
        {
            // 1) Ensure directory exists
            std::filesystem::path p(dbPath);
            if (!p.parent_path().empty())
                std::filesystem::create_directories(p.parent_path());

            // 2) Open DB
            sqlite3* db = nullptr;
            if (sqlite3_open(dbPath.c_str(), &db) != SQLITE_OK)
            {
                std::cerr << "Cannot open DB: " << sqlite3_errmsg(db) << "\n";
                return -1;
            }

            // 3) Begin transaction for speed
            sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);

            // 4) Prepare INSERT statement
            const char* sql    = R"(
    INSERT INTO RawPoints
      (id_sz, id_key, Easting, Northing, Altitude, Zone, Classification, NearbyPoints)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?);
)";
            sqlite3_stmt* stmt = nullptr;
            if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
            {
                std::cerr << "Prepare failed: " << sqlite3_errmsg(db) << "\n";
                sqlite3_close(db);
                return -1;
            }

            // double r2 = neighborRadius * neighborRadius;
            size_t N = rows.size();

            // 5) Loop rows
            for (size_t i = 0; i < N; ++i)
            {
                const auto& pt = rows[i];

                // 5a) Find neighbors on the fly
                // std::ostringstream ns;
                // bool first = true;
                // for (size_t j = 0; j < N; ++j)
                // {
                //     if (i == j)
                //         continue;
                //     double de = rows[j].easting - pt.easting;
                //     double dn = rows[j].northing - pt.northing;
                //     if (de * de + dn * dn <= r2)
                //     {
                //         if (!first)
                //             ns << ",";
                //         ns << rows[j].pointId;
                //         first = false;
                //     }
                // }
                // std::string neighStr = ns.str();

                // 5b) Bind parameters (1-based)
                sqlite3_bind_text(stmt, 1, pt.id.c_str(), -1, SQLITE_STATIC);
                sqlite3_bind_int64(stmt, 2, pt.pointId);
                sqlite3_bind_double(stmt, 3, pt.easting);
                sqlite3_bind_double(stmt, 4, pt.northing);
                sqlite3_bind_double(stmt, 5, pt.altitude);

                // zone as e.g. "15N" or "12T"
                std::string zoneStr = std::to_string(pt.zone.first) + pt.zone.second;
                sqlite3_bind_text(stmt, 6, zoneStr.c_str(), -1, SQLITE_STATIC);

                sqlite3_bind_text(stmt, 7, pt.classification.c_str(), -1, SQLITE_STATIC);
                sqlite3_bind_text(stmt, 8, "", -1, SQLITE_STATIC);

                // 5c) Execute & reset
                if (sqlite3_step(stmt) != SQLITE_DONE)
                {
                    // std::cerr << "Insert failed (pt " << pt.pointId << "): " << sqlite3_errmsg(db) << "\n";
                }
                sqlite3_reset(stmt);

                // Every 10000 rows, commit and begin a new transaction
                if ((i + 1) % 10000 == 0)
                {
                    sqlite3_exec(db, "END TRANSACTION;", nullptr, nullptr, nullptr);
                    sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);
                    // std::cout << "Inserted " << i + 1 << " rows into RawPoints\n";
                }
            }

            // 6) Finalize & commit
            sqlite3_finalize(stmt);
            sqlite3_exec(db, "END TRANSACTION;", nullptr, nullptr, nullptr);
            sqlite3_close(db);

            std::cout << "Inserted " << N << " rows into RawPoints\n";
            return 0;
        }

        /******************************************************************************
         * @brief Prints a PointRow to the console
         *
         * @param row - The PointRow to print
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-05-20
         ******************************************************************************/
        static void printPointRow(const PointRow& row)
        {
            std::cout << "  PointRow: "
                      << "Easting: " << row.easting << ", Northing: " << row.northing << ", Altitude: " << row.altitude << ", UTM Zone: " << row.zone.first
                      << row.zone.second << ", Classification: " << row.classification << "\n";
        }

        /******************************************************************************
         * @brief Loads and exports data from a LAS file
         *
         * @param filename - The name of the LAS file to load
         * @param idOffset - The offset to add to the point IDs (default is 0)
         * @return unsigned long long - The number of points loaded
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-05-20
         ******************************************************************************/
        unsigned long long loadAndExportData(const std::string& filename, unsigned long long idOffset = 0)
        {
            // Load data from the file
            std::ifstream in(filename, std::ios::binary);
            if (!in)
            {
                throw std::runtime_error("Failed to open file: " + filename);
            }

            // Read the header
            MinimalLASHeader h = readMinimalHeader(in);

            // Read the VLRs
            in.seekg(h.headerSize, std::ios::beg);
            for (uint32_t i = 0; i < h.numVLRs; ++i)
            {
                VLRHeader vh;
                in.read(reinterpret_cast<char*>(&vh), sizeof(vh));
                std::string uid  = trim(vh.userID, sizeof(vh.userID));
                std::string desc = trim(vh.description, sizeof(vh.description));
                std::vector<char> payload(vh.recordLengthAfterHeader);
                in.read(payload.data(), payload.size());
            }

            // Extract UTM zone from VLR #1
            auto [zone, hemi] = extractUTMZoneFromVLR1(in, h);
            if (zone <= 0)
            {
                std::cerr << "Failed to find UTM zone in VLR #1\n";
            }

            // Collect point records
            auto pointRows = collectPointRecords(in, h, {zone, hemi}, idOffset);

            bool foundAll  = pointRows.size() == h.numberOfPointRecords;
            if (foundAll)
            {
                std::cout << "Found all points: " << pointRows.size() << "\n";

                // Populate neighbors
                PopulateSQL(pointRows, 5.0);
            }
            else
            {
                std::cerr << "Warning: Found only " << pointRows.size() << " points out of " << h.numberOfPointRecords << "\n";
                return -1;
            }

            return pointRows.size();
        }
};

/******************************************************************************
 * @brief Overload the << operator for LiDARVersion1_4::PointClassification
 *
 * @param os - The output stream
 * @param cls - The PointClassification enum value
 * @return std::ostream& - The output stream
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-21
 ******************************************************************************/
std::ostream& operator<<(std::ostream& os, LiDARVersion1_4::PointClassification cls)
{
    return os << LiDARVersion1_4::to_string(cls);
}

/******************************************************************************
 * @brief Main function for the LiDAR processing tool
 *
 * @param argc - The number of command line arguments
 * @param argv - The command line arguments
 * @return int - The exit code
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file.las> | <directory/>\n";
        return 1;
    }

    std::filesystem::path inputPath(argv[1]);
    if (!std::filesystem::exists(inputPath))
    {
        std::cerr << "Path does not exist: " << inputPath << "\n";
        return 1;
    }

    // Gather all .las files
    std::vector<std::filesystem::path> lasFiles;
    if (std::filesystem::is_directory(inputPath))
    {
        for (auto& entry : std::filesystem::directory_iterator(inputPath))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".las")
                lasFiles.push_back(entry.path());
        }
        if (lasFiles.empty())
        {
            std::cerr << "No .las files found in directory: " << inputPath << "\n";
            return 1;
        }
    }
    else
    {
        if (inputPath.extension() != ".las")
        {
            std::cerr << "Not a .las file: " << inputPath << "\n";
            return 1;
        }
        lasFiles.push_back(inputPath);
    }

    LiDARVersion1_4 lidar;

    // Start overall timer
    auto t0                     = std::chrono::steady_clock::now();

    unsigned long long idOffset = 0;

    for (auto const& lasPath : lasFiles)
    {
        std::cout << "Processing: " << lasPath << " ...\n";

        // Start per-file timer
        auto f0 = std::chrono::steady_clock::now();

        try
        {
            idOffset += lidar.loadAndExportData(lasPath.string(), idOffset);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error processing " << lasPath << ": " << e.what() << "\n";
            continue;
        }

        // End per-file timer
        auto f1                            = std::chrono::steady_clock::now();
        std::chrono::duration<double> fdur = f1 - f0;
        std::cout << "  Done in " << fdur.count() << " seconds\n\n";
    }

    // End overall timer
    auto t1                                = std::chrono::steady_clock::now();
    std::chrono::duration<double> totalDur = t1 - t0;
    std::cout << "Total processing time: " << totalDur.count() << " seconds for " << lasFiles.size() << " file(s)\n";

    return 0;
}
