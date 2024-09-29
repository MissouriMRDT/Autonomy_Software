/******************************************************************************
 * @brief A program to read a LAS file and decode the header and Point Data
 *        Records Format 6.
 *
 *        This program reads a LAS (LiDAR data) file, parses the header, and reads
 *        point data records of format 6 according to the LAS 1.4 specification.
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
#include <fstream>
#include <iostream>

/// \endcond

#define PRINT_POINT_RECORDS false    ///< Set to true to enable point data output

// Ensure that the structures are packed without any padding bytes
#pragma pack(push, 1)

/******************************************************************************
 * @brief Represents the header of a LAS file.
 *
 *        This structure maps directly to the binary layout specified in the
 *        LAS 1.4 format. Each field corresponds to a specific part of the LAS
 *        file header.
 *
 * @struct LASHeader
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-28
 ******************************************************************************/
struct LASHeader
{
        char fileSignature[4];                             ///< File signature; should be "LASF".
        uint16_t fileSourceID;                             ///< File source identifier.
        uint16_t globalEncoding;                           ///< Global encoding bit field.
        uint32_t projectID_GUID_data1;                     ///< Project ID GUID data 1.
        uint16_t projectID_GUID_data2;                     ///< Project ID GUID data 2.
        uint16_t projectID_GUID_data3;                     ///< Project ID GUID data 3.
        uint8_t projectID_GUID_data4[8];                   ///< Project ID GUID data 4.
        uint8_t versionMajor;                              ///< Major version number.
        uint8_t versionMinor;                              ///< Minor version number.
        char systemIdentifier[32];                         ///< System identifier string.
        char generatingSoftware[32];                       ///< Generating software description.
        uint16_t fileCreationDayOfYear;                    ///< Day of year of file creation.
        uint16_t fileCreationYear;                         ///< Year of file creation.
        uint16_t headerSize;                               ///< Size of the header in bytes.
        uint32_t offsetToPointData;                        ///< Offset to the point data records.
        uint32_t numberOfVariableLengthRecords;            ///< Number of variable length records.
        uint8_t pointDataFormatID;                         ///< Point data format ID (should be 6).
        uint16_t pointDataRecordLength;                    ///< Length of each point data record.
        uint32_t legacyNumberOfPointRecords;               ///< Legacy number of point records.
        uint32_t legacyNumberOfPointsByReturn[5];          ///< Legacy points by return counts.
        double xScaleFactor;                               ///< X coordinate scale factor.
        double yScaleFactor;                               ///< Y coordinate scale factor.
        double zScaleFactor;                               ///< Z coordinate scale factor.
        double xOffset;                                    ///< X coordinate offset.
        double yOffset;                                    ///< Y coordinate offset.
        double zOffset;                                    ///< Z coordinate offset.
        double maxX;                                       ///< Maximum X coordinate.
        double minX;                                       ///< Minimum X coordinate.
        double maxY;                                       ///< Maximum Y coordinate.
        double minY;                                       ///< Minimum Y coordinate.
        double maxZ;                                       ///< Maximum Z coordinate.
        double minZ;                                       ///< Minimum Z coordinate.
        uint64_t startOfWaveformDataPacketRecord;          ///< Start of waveform data packet record.
        uint64_t startOfExtendedVariableLengthRecords;     ///< Start of extended variable length records.
        uint32_t numberOfExtendedVariableLengthRecords;    ///< Number of extended variable length records.
        uint64_t numberOfPointRecords;                     ///< Total number of point records (for LAS 1.4).
        uint64_t numberOfPointsByReturn[15];               ///< Number of points by return counts (for LAS 1.4).
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
        int32_t X;                                 ///< X coordinate as a signed 32-bit integer.
        int32_t Y;                                 ///< Y coordinate as a signed 32-bit integer.
        int32_t Z;                                 ///< Z coordinate as a signed 32-bit integer.
        uint16_t intensity;                        ///< Intensity value of the point.
        uint8_t returnNumberAndNumberOfReturns;    ///< Combined field for return number and number of returns.
        uint8_t classificationFlagsAndOthers;      ///< Combined field for classification flags and other bits.
        uint8_t classification;                    ///< Point classification.
        uint8_t userData;                          ///< User data.
        int16_t scanAngle;                         ///< Scan angle rank.
        uint16_t pointSourceID;                    ///< Point source ID.
        double gpsTime;                            ///< GPS time of the point.
};

// Restore the previous packing alignment
#pragma pack(pop)

/******************************************************************************
 * @brief Main function that reads the LAS file and processes the data.
 *
 * @return int - Returns 0 on success, non-zero on failure.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-09-28
 ******************************************************************************/
int main()
{
    // Open the LAS file in binary mode
    std::ifstream infile("../../data/USGS_Data/utah/USGS_LPC_UT_StatewideSouth_2020_A20_12SWH_URC_2KM.las", std::ios::binary);
    if (!infile)
    {
        // Output an error message if the file cannot be opened
        std::cerr << "Cannot open input.las file." << std::endl;
        return 1;
    }

    // Create an instance of LASHeader to hold the header data
    LASHeader header;

    // Read the header from the file
    infile.read(reinterpret_cast<char*>(&header), sizeof(LASHeader));

    // Verify the file signature to ensure it's a valid LAS file
    if (strncmp(header.fileSignature, "LASF", 4) != 0)
    {
        // Output an error message if the file signature is invalid
        std::cerr << "Invalid LAS file." << std::endl;
        return 1;
    }

    // Display header information to the console
    std::cout << "LAS File Header:" << std::endl;
    std::cout << "File Signature: " << std::string(header.fileSignature, 4) << std::endl;
    std::cout << "Version: " << static_cast<int>(header.versionMajor) << "." << static_cast<int>(header.versionMinor) << std::endl;
    std::cout << "System Identifier: " << std::string(header.systemIdentifier, 32) << std::endl;
    std::cout << "Generating Software: " << std::string(header.generatingSoftware, 32) << std::endl;
    std::cout << "Point Data Format ID: " << static_cast<int>(header.pointDataFormatID) << std::endl;
    std::cout << "Number of Point Records: " << header.numberOfPointRecords << std::endl;

    // Check if the Point Data Format ID is 6
    if (header.pointDataFormatID != 6)
    {
        // Output an error message if the point data format is not 6
        std::cerr << "Point Data Format ID is not 6." << std::endl;
        return 1;
    }

    // Ensure that the point data record length is sufficient for format 6
    if (header.pointDataRecordLength < sizeof(PointDataRecordFormat6))
    {
        // Output an error message if the point data record length is too small
        std::cerr << "Point Data Record Length is smaller than expected." << std::endl;
        return 1;
    }

    // Move the file read position to the start of the point data records
    infile.seekg(header.offsetToPointData, std::ios::beg);

    // Create an instance of PointDataRecordFormat6 to hold the point data
    PointDataRecordFormat6 point;

    // Calculate the number of extra bytes in each point data record
    size_t extraBytes = header.pointDataRecordLength - sizeof(PointDataRecordFormat6);

    // Pointer to hold any extra data in the point data records
    char* extraData = nullptr;

    // If there are extra bytes, allocate memory to hold them
    if (extraBytes > 0)
    {
        extraData = new char[extraBytes];
    }

    // Loop through each point data record in the file
    for (uint64_t i = 0; i < header.numberOfPointRecords; ++i)
    {
        // Read the point data record from the file
        infile.read(reinterpret_cast<char*>(&point), sizeof(PointDataRecordFormat6));

        // Check if the read operation was successful
        if (!infile)
        {
            // Output an error message if there was an error reading the point data
            std::cerr << "Error reading point data." << std::endl;
            break;
        }

        // If there are extra bytes, read them into the extraData buffer
        if (extraBytes > 0)
        {
            infile.read(extraData, extraBytes);
            if (!infile)
            {
                // Output an error message if there was an error reading the extra data
                std::cerr << "Error reading extra point data." << std::endl;
                break;
            }
        }

        // Calculate the actual X, Y, and Z coordinates by applying the scale factor and offset
        double x = point.X * header.xScaleFactor + header.xOffset;
        double y = point.Y * header.yScaleFactor + header.yOffset;
        double z = point.Z * header.zScaleFactor + header.zOffset;

        // Extract the return number (bits 0-3) from the combined field
        uint8_t returnNumber = point.returnNumberAndNumberOfReturns & 0x0F;
        // Extract the number of returns (bits 4-7) from the combined field
        uint8_t numberOfReturns = (point.returnNumberAndNumberOfReturns >> 4) & 0x0F;

        // Extract the classification flags (bits 0-3) from the combined field
        uint8_t classificationFlags = point.classificationFlagsAndOthers & 0x0F;
        // Extract the scanner channel (bits 4-5) from the combined field
        uint8_t scannerChannel = (point.classificationFlagsAndOthers >> 4) & 0x03;
        // Extract the scan direction flag (bit 6) from the combined field
        uint8_t scanDirectionFlag = (point.classificationFlagsAndOthers >> 6) & 0x01;
        // Extract the edge of flight line flag (bit 7) from the combined field
        uint8_t edgeOfFlightLine = (point.classificationFlagsAndOthers >> 7) & 0x01;

        // Display point data if enabled
        if (PRINT_POINT_RECORDS)
        {
            std::cout << "Point " << i + 1 << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
            std::cout << "  Intensity: " << point.intensity << std::endl;
            std::cout << "  Return Number: " << static_cast<int>(returnNumber) << ", Number of Returns: " << static_cast<int>(numberOfReturns) << std::endl;
            std::cout << "  Classification: " << static_cast<int>(point.classification) << std::endl;
            std::cout << "  GPS Time: " << point.gpsTime << std::endl;
        }
    }

    // Deallocate the memory used for extra data
    delete[] extraData;

    // Close the input file
    infile.close();

    return 0;
}
