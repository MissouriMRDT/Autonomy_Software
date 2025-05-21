/******************************************************************************
 * @brief Implementation of the LiDAR runtime query interface.
 *
 * @file LiDARHandler.cpp
 * @author Eli Byrd
 * @date 2025-05-20
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "LiDARHandler.h"
#include "../AutonomyLogging.h"
#include <cmath>
#include <iostream>

/******************************************************************************
 * @brief Destroy the LiDARHandler::LiDARHandler object.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
LiDARHandler::~LiDARHandler()
{
    Finalize();
}

/******************************************************************************
 * @brief Initializes the LiDARHandler by opening the SQLite database and preparing the query.
 *
 * This method opens the specified SQLite database file and prepares the internal
 * SQL statement used to query nearby point records. It must be called before any
 * queries are made using GetNearbyPoints().
 *
 * @param dbPath Relative or absolute path to the SQLite database file. If a relative path
 *               is provided, it must be relative to the directory from which the
 *               final executable is launched (i.e., the current working directory).
 *
 * @return bool - true if initialization and statement preparation succeed; false otherwise.
 *
 * @note If the database file cannot be found or accessed, an error message will be printed
 *       to std::cerr and this function will return false.
 *
 * @author Eli Byrd
 * @date 2025-05-20
 ******************************************************************************/
bool LiDARHandler::Initialize(const std::string& szDBPath)
{
    // Attempt to open the SQLite database, if it fails, print the error message and return false
    if (sqlite3_open(szDBPath.c_str(), &m_sqlDatabase) != SQLITE_OK)
    {
        LOG_CRITICAL(logging::g_qSharedLogger, "Failed to open database: {}", sqlite3_errmsg(m_sqlDatabase));
        return false;
    }

    // Prepare the SQL statement for querying nearby points
    return PrepareNearbyStatement();
}

/******************************************************************************
 * @brief Prepares the SQL statement used to query nearby LiDAR points.
 *
 * This function constructs and compiles a parameterized SQL query that retrieves
 * all point records from the `RawPoints` table within a bounding box defined by
 * a center coordinate and a search radius. The actual values for easting,
 * northing, and radius are bound later at runtime in GetNearbyPoints().
 *
 * The bounding box is defined using:
 *   - Easting BETWEEN (easting - radius) AND (easting + radius)
 *   - Northing BETWEEN (northing - radius) AND (northing + radius)
 *
 * @return true if the SQL statement is successfully prepared; false otherwise.
 *         On failure, an error message will be printed to std::cerr.
 *
 * @note This method is called automatically during Initialize().
 * @note The compiled statement is cached for repeated use.
 *
 * @author Eli Byrd
 * @date 2025-05-20
 ******************************************************************************/
bool LiDARHandler::PrepareNearbyStatement()
{
    // Create the SQL statement
    const char* szSQLStatement = R"(
         SELECT id, Easting, Northing, Altitude, Zone, Classification
         FROM RawPoints
         WHERE Easting BETWEEN (? - ?) AND (? + ?)
           AND Northing BETWEEN (? - ?) AND (? + ?);
     )";

    // Prepare the SQL statement, if it fails, print the error message and return false
    if (sqlite3_prepare_v2(m_sqlDatabase, szSQLStatement, -1, &m_sqlStatement, nullptr) != SQLITE_OK)
    {
        LOG_CRITICAL(logging::g_qSharedLogger, "Failed to prepare SQL statement: {}", sqlite3_errmsg(m_sqlDatabase));
        return false;
    }

    // Return true to indicate success, statement has been prepared and stored
    return true;
}

/******************************************************************************
 * @brief Retrieves all LiDAR points within a specified radius of a given coordinate.
 *
 * This method uses a precompiled SQL statement to query the `RawPoints` table for
 * all records whose (Easting, Northing) coordinates fall within a square bounding box
 * centered at the given input location and extended by `radiusMeters` in all directions.
 *
 * The bounding box logic is implemented in SQL using:
 *   - Easting BETWEEN (easting - radius) AND (easting + radius)
 *   - Northing BETWEEN (northing - radius) AND (northing + radius)
 *
 * @param easting The UTM easting coordinate in meters.
 * @param northing The UTM northing coordinate in meters.
 * @param radiusMeters The radius in meters to search within. Default is 5.0 meters.
 * @return std::vector<PointRow> A vector of points located within the bounding box.
 *
 * @note This performs a square bounding box query, not an exact circular distance check.
 *       If exact radial filtering is needed, post-process the returned points using
 *       Euclidean distance.
 *
 * @warning Ensure `Initialize()` has been successfully called before using this method.
 *
 * @author Eli Byrd
 * @date 2025-05-20
 ******************************************************************************/
std::vector<LiDARHandler::PointRow> LiDARHandler::GetNearbyPoints(double dEasting, double dNorthing, double dRadiusMeters)
{
    // Create a vector to hold the results
    std::vector<PointRow> results;

    // Reset the prepared statement and clear any previous bindings
    sqlite3_reset(m_sqlStatement);
    sqlite3_clear_bindings(m_sqlStatement);

    // Bind the parameters to the SQL statement
    sqlite3_bind_double(m_sqlStatement, 1, dEasting);
    sqlite3_bind_double(m_sqlStatement, 2, dRadiusMeters);
    sqlite3_bind_double(m_sqlStatement, 3, dEasting);
    sqlite3_bind_double(m_sqlStatement, 4, dRadiusMeters);
    sqlite3_bind_double(m_sqlStatement, 5, dNorthing);
    sqlite3_bind_double(m_sqlStatement, 6, dRadiusMeters);
    sqlite3_bind_double(m_sqlStatement, 7, dNorthing);
    sqlite3_bind_double(m_sqlStatement, 8, dRadiusMeters);

    // Execute the SQL statement and iterate through the results
    while (sqlite3_step(m_sqlStatement) == SQLITE_ROW)
    {
        // Create a new PointRow object
        PointRow row;

        // Populate the PointRow object with data from the current row
        row.nId              = sqlite3_column_int(m_sqlStatement, 0);
        row.dEasting         = sqlite3_column_double(m_sqlStatement, 1);
        row.dNorthing        = sqlite3_column_double(m_sqlStatement, 2);
        row.dAltitude        = sqlite3_column_double(m_sqlStatement, 3);
        row.szZone           = reinterpret_cast<const char*>(sqlite3_column_text(m_sqlStatement, 4));
        row.szClassification = reinterpret_cast<const char*>(sqlite3_column_text(m_sqlStatement, 5));

        // Add the populated PointRow object to the results vector
        results.push_back(row);
    }

    // Return the resulting vector of points
    return results;
}

/******************************************************************************
 * @brief Finalizes and closes internal SQLite resources.
 *
 * This method safely deallocates all SQLite resources used by the handler:
 * - Finalizes the prepared statement used for nearby queries.
 * - Closes the SQLite database connection.
 *
 * It should be called before destroying the `LiDARHandler` instance or when
 * the handler is no longer needed. Calling this method multiple times is safe.
 *
 * @note Automatically called by the destructor, but may also be used explicitly
 *       in controlled lifetime scenarios.
 *
 * @author Eli Byrd
 * @date 2025-05-20
 ******************************************************************************/
void LiDARHandler::Finalize()
{
    // Check if the statement is prepared and finalize it
    if (m_sqlStatement)
    {
        sqlite3_finalize(m_sqlStatement);
        m_sqlStatement = nullptr;
    }

    // Check if the database is open and close it
    if (m_sqlDatabase)
    {
        sqlite3_close(m_sqlDatabase);
        m_sqlDatabase = nullptr;
    }
}
