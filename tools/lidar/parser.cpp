/******************************************************************************
 * @brief A program to read multiple LAS files, decode the stHeader and Point Data
 *        Records Format 6, and merge the data into 10m x 10m grid files based
 *        on UTM coordinates.
 *
 *        This program reads LAS (LiDAR data) files, parses the stHeaders, and reads
 *        point data records of format 6 according to the LAS 1.4 specification,
 *        then merges the point data for each UTM grid from multiple files and
 *        writes a single output file for each 10m x 10m grid.
 *
 * @file parser.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-28
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

/// \cond
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

/// \endcond

#define DEBUG    // Comment this line to disable debug prints
#ifdef DEBUG
#define DEBUG_PRINT(x) std::cout << x << std::endl
#else
#define DEBUG_PRINT(x)
#endif

// Ensure that the structures are packed without any padding bytes
#pragma pack(push, 1)

/******************************************************************************
 * @brief Represents the stHeader of a LAS file.
 *
 *        This structure maps directly to the binary layout specified in the
 *        LAS 1.4 format. Each field corresponds to a specific part of the LAS
 *        file stHeader.
 *
 * @struct LASHeader
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-28
 ******************************************************************************/
struct LASHeader
{
        char cFileSignature[4];                              // File signature; should be "LASF".
        uint16_t unFileSourceID;                             // File source identifier.
        uint16_t unGlobalEncoding;                           // Global encoding bit field.
        uint32_t unProjectID_GUID_data1;                     // Project ID GUID data 1.
        uint16_t unProjectID_GUID_data2;                     // Project ID GUID data 2.
        uint16_t unProjectID_GUID_data3;                     // Project ID GUID data 3.
        uint8_t unProjectID_GUID_data4[8];                   // Project ID GUID data 4.
        uint8_t unVersionMajor;                              // Major version number.
        uint8_t unVersionMinor;                              // Minor version number.
        char cSystemIdentifier[32];                          // System identifier string.
        char cGeneratingSoftware[32];                        // Generating software description.
        uint16_t unFileCreationDayOfYear;                    // Day of year of file creation.
        uint16_t unFileCreationYear;                         // Year of file creation.
        uint16_t unHeaderSize;                               // Size of the stHeader in bytes.
        uint32_t unOffsetToPointData;                        // Offset to the point data records.
        uint32_t unNumberOfVariableLengthRecords;            // Number of variable length records.
        uint8_t unPointDataFormatID;                         // Point data format ID (should be 6).
        uint16_t unPointDataRecordLength;                    // Length of each point data record.
        uint32_t unLegacyNumberOfPointRecords;               // Legacy number of point records.
        uint32_t unLegacyNumberOfPointsByReturn[5];          // Legacy points by return counts.
        double dXScaleFactor;                                // X coordinate scale factor.
        double dYScaleFactor;                                // Y coordinate scale factor.
        double dZScaleFactor;                                // Z coordinate scale factor.
        double dXOffset;                                     // X coordinate offset.
        double dYOffset;                                     // Y coordinate offset.
        double dZOffset;                                     // Z coordinate offset.
        double dMaxX;                                        // Maximum X coordinate.
        double dMinX;                                        // Minimum X coordinate.
        double dMaxY;                                        // Maximum Y coordinate.
        double dMinY;                                        // Minimum Y coordinate.
        double dMaxZ;                                        // Maximum Z coordinate.
        double dMinZ;                                        // Minimum Z coordinate.
        uint64_t unStartOfWaveformDataPacketRecord;          // Start of waveform data packet record.
        uint64_t unStartOfExtendedVariableLengthRecords;     // Start of extended variable length records.
        uint32_t unNumberOfExtendedVariableLengthRecords;    // Number of extended variable length records.
        uint64_t unNumberOfPointRecords;                     // Total number of point records (for LAS 1.4).
        uint64_t unNumberOfPointsByReturn[15];               // Number of points by return counts (for LAS 1.4).
};

/******************************************************************************
 * @brief Represents a point data record of format 6.
 *
 *        Contains the point's position and various attributes according to the
 *        LAS 1.4 specification.
 *
 * @struct PointDataRecordFormat6
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-28
 ******************************************************************************/
struct PointDataRecordFormat6
{
        int32_t unXCoord;                            // X coordinate as a signed 32-bit integer.
        int32_t unYCoord;                            // Y coordinate as a signed 32-bit integer.
        int32_t unZCoord;                            // Z coordinate as a signed 32-bit integer.
        uint16_t unIntensity;                        // Intensity value of the point.
        uint8_t unReturnNumberAndNumberOfReturns;    // Combined field for return number and number of returns.
        uint8_t unClassificationFlagsAndOthers;      // Combined field for classification flags and other bits.
        uint8_t unClassification;                    // Point classification.
        uint8_t unUserData;                          // User data.
        int16_t unScanAngle;                         // Scan angle rank.
        uint16_t unPointSourceID;                    // Point source ID.
        double dGpsTime;                             // GPS time of the point.
};

// Restore the previous packing alignment
#pragma pack(pop)

/******************************************************************************
 * @brief Parses the LAS stHeader from the input file.
 *
 *        Reads the LAS stHeader and verifies the file signature and point format.
 *        Adds debug information about the file's metadata.
 *
 * @param fInfile - Input file stream of the LAS file.
 * @param stHeader - Reference to LASHeader struct where the parsed data will be stored.
 * @return true  - If the stHeader was successfully parsed.
 * @return false - If the stHeader was not successfully parsed.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-30
 ******************************************************************************/
bool ParseLASHeader(std::ifstream& fInfile, LASHeader& stHeader)
{
    fInfile.read(reinterpret_cast<char*>(&stHeader), sizeof(LASHeader));

    if (strncmp(stHeader.cFileSignature, "LASF", 4) != 0)
    {
        std::cerr << "Invalid LAS file." << std::endl;
        return false;
    }

    if (stHeader.unPointDataFormatID != 6)
    {
        std::cerr << "Point Data Format ID is not 6." << std::endl;
        return false;
    }

    DEBUG_PRINT("LAS file parsed successfully. Point Data Format: " + std::to_string(static_cast<int>(stHeader.unPointDataFormatID)));
    DEBUG_PRINT("Number of Point Records: " + std::to_string(stHeader.unNumberOfPointRecords));

    return true;
}

/******************************************************************************
 * @brief Processes the LAS points and organizes them into a 10m x 10m grid.
 *
 *        Adds debug output to track the number of points processed and their
 *        corresponding UTM grid coordinates.
 *
 * @param fInfile - Input file stream of the LAS file.
 * @param stHeader - The parsed LASHeader struct containing metadata of the LAS file.
 * @return std::map<std::pair<int, int>, std::vector<PointDataRecordFormat6>> - A map that associates grid coordinates to the corresponding points.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-30
 ******************************************************************************/
std::map<std::pair<int, int>, std::vector<PointDataRecordFormat6>> ProcessLASPoints(std::ifstream& fInfile, const LASHeader& stHeader)
{
    std::map<std::pair<int, int>, std::vector<PointDataRecordFormat6>> mGridPoints;

    size_t siExtraBytes = stHeader.unPointDataRecordLength - sizeof(PointDataRecordFormat6);
    char* cExtraData    = nullptr;
    if (siExtraBytes > 0)
    {
        cExtraData = new char[siExtraBytes];
    }

    PointDataRecordFormat6 stPoint;
    fInfile.seekg(stHeader.unOffsetToPointData, std::ios::beg);

    DEBUG_PRINT("Processing point data...");

    for (uint64_t i = 0; i < stHeader.unNumberOfPointRecords; ++i)
    {
        fInfile.read(reinterpret_cast<char*>(&stPoint), sizeof(PointDataRecordFormat6));
        if (!fInfile)
        {
            std::cerr << "Error reading point data." << std::endl;
            break;
        }

        if (siExtraBytes > 0)
        {
            fInfile.read(cExtraData, siExtraBytes);
            if (!fInfile)
            {
                std::cerr << "Error reading extra point data." << std::endl;
                break;
            }
        }

        double dScaledXCoord = stPoint.unXCoord * stHeader.dXScaleFactor + stHeader.dXOffset;
        double dScaledYCoord = stPoint.unYCoord * stHeader.dYScaleFactor + stHeader.dYOffset;

        int nGridX           = static_cast<int>((dScaledXCoord - stHeader.dMinX) / 10.0);
        int nGridY           = static_cast<int>((dScaledYCoord - stHeader.dMinY) / 10.0);

        mGridPoints[std::make_pair(nGridX, nGridY)].push_back(stPoint);
    }

    DEBUG_PRINT("Finished processing " + std::to_string(mGridPoints.size()) + " grid cells.");

    delete[] cExtraData;

    return mGridPoints;
}

/******************************************************************************
 * @brief Merges points for the same grid from multiple LAS files into one output file.
 *
 *        Adds debug output for each file written and prints the number of points
 *        in each grid file.
 *
 * @param mAllGridPoints - A map storing points from all LAS files for each UTM grid.
 * @param szOutputPrefix - Prefix for the output LAS files.
 * @param szOutputDir - Directory where the output LAS files will be written.
 * @param stHeader - The LAS stHeader from the first file.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-30
 ******************************************************************************/
void WriteMergedGridFiles(const std::map<std::pair<int, int>, std::vector<PointDataRecordFormat6>>& mAllGridPoints,
                          const std::string& szOutputPrefix,
                          const std::string& szOutputDir,
                          const LASHeader& stHeader)
{
    int nTotalFilesWritten = 0;

    for (const std::pair<const std::pair<int, int>, std::vector<PointDataRecordFormat6>>& prGrid : mAllGridPoints)
    {
        double dGridMinX       = stHeader.dMinX + prGrid.first.first * 10.0;
        double dGridMinY       = stHeader.dMinY + prGrid.first.second * 10.0;

        std::string szFilename = szOutputDir                                      // The directory where the output LAS files will be saved
                                 + "/"                                            // Add a separator between the directory and the file name
                                 + szOutputPrefix                                 // The prefix for the output file (e.g., "USGS_Grid")
                                 + "_utm_"                                        // Label indicating the UTM coordinate grid
                                 + std::to_string(static_cast<int>(dGridMinX))    // Convert the minimum X coordinate of the grid to a string
                                 + "_"                                            // Separator between the X and Y coordinates
                                 + std::to_string(static_cast<int>(dGridMinY))    // Convert the minimum Y coordinate of the grid to a string
                                 + ".las";                                        // The file extension for LAS files

        std::ofstream fOutfile(szFilename, std::ios::binary);

        LASHeader gridHeader              = stHeader;
        gridHeader.dMinX                  = dGridMinX;
        gridHeader.dMaxX                  = dGridMinX + 10.0;
        gridHeader.dMinY                  = dGridMinY;
        gridHeader.dMaxY                  = dGridMinY + 10.0;
        gridHeader.unNumberOfPointRecords = prGrid.second.size();

        fOutfile.write(reinterpret_cast<const char*>(&gridHeader), sizeof(LASHeader));

        for (const PointDataRecordFormat6& stPoint : prGrid.second)
        {
            fOutfile.write(reinterpret_cast<const char*>(&stPoint), sizeof(PointDataRecordFormat6));
        }

        fOutfile.close();
        nTotalFilesWritten++;
        DEBUG_PRINT("Written grid file: " + szFilename + " with " + std::to_string(prGrid.second.size()) + " points.");
    }

    DEBUG_PRINT("Total grid files written: " + std::to_string(nTotalFilesWritten));
}

/******************************************************************************
 * @brief Processes multiple LAS files and merges them into 10m x 10m grid files.
 *
 *        Adds debug output for each LAS file being processed.
 *
 * @param vLasFiles - Vector of paths to input LAS files.
 * @param szOutputPrefix - Prefix for the output LAS files.
 * @param szOutputDir - Directory where the output LAS files will be written.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-30
 ******************************************************************************/
void ProcessAndMergeLASFiles(const std::vector<std::string>& vLasFiles, const std::string& szOutputPrefix, const std::string& szOutputDir)
{
    if (!std::filesystem::exists(szOutputDir))
    {
        std::filesystem::create_directories(szOutputDir);
    }

    std::map<std::pair<int, int>, std::vector<PointDataRecordFormat6>> mAllGridPoints;
    LASHeader stReferenceHeader;

    // Process each LAS file
    for (const std::string& szLasFile : vLasFiles)
    {
        DEBUG_PRINT("Processing LAS file: " + szLasFile);

        std::ifstream fInfile(szLasFile, std::ios::binary);
        if (!fInfile)
        {
            std::cerr << "Cannot open LAS file: " << szLasFile << std::endl;
            continue;
        }

        LASHeader stHeader;
        if (!ParseLASHeader(fInfile, stHeader))
        {
            std::cerr << "Failed to parse LAS stHeader for: " << szLasFile << std::endl;
            continue;
        }

        if (mAllGridPoints.empty())
        {
            stReferenceHeader = stHeader;    // Use the stHeader from the first file as the reference
        }

        std::map<std::pair<int, int>, std::vector<PointDataRecordFormat6>> mGridPoints = ProcessLASPoints(fInfile, stHeader);

        // Merge points into a common map to avoid overwriting
        for (const std::pair<const std::pair<int, int>, std::vector<PointDataRecordFormat6>>& prGrid : mGridPoints)
        {
            mAllGridPoints[prGrid.first].insert(mAllGridPoints[prGrid.first].end(), prGrid.second.begin(), prGrid.second.end());
        }

        fInfile.close();
    }

    // Write merged points into final grid files
    WriteMergedGridFiles(mAllGridPoints, szOutputPrefix, szOutputDir, stReferenceHeader);
}

/******************************************************************************
 * @brief Main function to merge multiple LAS files that border each other and
 *        may have overlapping regions.
 *
 *        Adds a summary of the total number of grid files written.
 *
 * @return int - Returns 0 on successful execution, other on failure.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-28
 ******************************************************************************/
int main()
{
    // List of LAS files to process
    std::vector<std::string> vLasFiles = {
        "../../data/USGS_Data/utah/USGS_LPC_UT_StatewideSouth_2020_A20_12SWH_URC_1KM_NW.las",    // URC Northwest 1KM
        "../../data/USGS_Data/utah/USGS_LPC_UT_StatewideSouth_2020_A20_12SWH_URC_1KM_NE.las",    // URC Northeast 1KM
        "../../data/USGS_Data/utah/USGS_LPC_UT_StatewideSouth_2020_A20_12SWH_URC_1KM_SW.las",    // URC Southwest 1KM
        "../../data/USGS_Data/utah/USGS_LPC_UT_StatewideSouth_2020_A20_12SWH_URC_1KM_SE.las"     // URC Southeast 1KM
    };

    // Output file prefix
    std::string szOutputPrefix = "USGS_Grid";

    // Output directory
    std::string szOutputDir = "./output_grids/URC";

    // Start processing and merging LAS files
    DEBUG_PRINT("Starting the LAS file processing and merging...");

    // Process and merge LAS files
    ProcessAndMergeLASFiles(vLasFiles, szOutputPrefix, szOutputDir);

    DEBUG_PRINT("Processing and merging completed.");

    return 0;
}
