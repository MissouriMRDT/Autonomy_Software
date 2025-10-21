/******************************************************************************
 * @brief Implementation of the LiDAR runtime query interface.
 *
 * @file LiDARHandler.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com), Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "LiDARHandler.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Construct a new LiDARHandler::LiDARHandler object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-13
 ******************************************************************************/
LiDARHandler::LiDARHandler()
{
    // Initialize member variables.
    m_pSQLDatabase  = nullptr;
    m_pSQLStatement = nullptr;
    m_bIsDBOpen     = false;
}

/******************************************************************************
 * @brief Destroy the LiDARHandler::LiDARHandler object.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com), Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 ******************************************************************************/
LiDARHandler::~LiDARHandler()
{
    this->CloseDB();    // Ensure the database is closed on destruction.
}

/******************************************************************************
 * @brief Initializes the LiDARHandler by opening the SQLite database and preparing the query.
 *
 * This method opens the specified SQLite database file and prepares the internal
 * SQL statement used to query nearby point records. It must be called before any
 * queries are made using GetNearbyPoints().
 *
 * @param szDBPath Relative or absolute path to the SQLite database file. If a relative path
 *               is provided, it must be relative to the directory from which the
 *               final executable is launched (i.e., the current working directory).
 *
 * @return true - If the database was successfully opened and the SQL statement prepared.
 * @return false - If there was an error opening the database or preparing the SQL statement.
 *
 * @note If the database file cannot be found or accessed, an error message will be printed
 *       to std::cerr and this function will return false.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-07-13
 ******************************************************************************/
bool LiDARHandler::OpenDB(const std::string& szDBPath)
{
    // Acquire a write lock on the mutex to ensure thread safety.
    std::unique_lock<std::shared_mutex> lkWriteLock(m_muQueryMutex);

    // Check if the database is already open.
    if (m_bIsDBOpen)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "LiDARHandler: Database is already open. Closing existing connection before opening a new one.");
        // Release lock before calling CloseDB to avoid deadlock.
        lkWriteLock.unlock();
        this->CloseDB();
        lkWriteLock.lock();
    }

    // Attempt to open the SQLite database.
    int nReturnCode = sqlite3_open(szDBPath.c_str(), &m_pSQLDatabase);
    if (nReturnCode != SQLITE_OK)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler: Failed to open database at '{}': {}", szDBPath, sqlite3_errmsg(m_pSQLDatabase));
        // Return false on failure.
        return false;
    }

    // Set the database open flag to true.
    m_bIsDBOpen = true;

    // Log success.
    LOG_INFO(logging::g_qSharedLogger, "LiDARHandler: Successfully opened database at '{}'.", szDBPath);

    return true;
}

/******************************************************************************
 * @brief Closes the currently open LiDAR database.
 *
 * @return true - If the database was successfully closed.
 * @return false - If there was an error closing the database.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-13
 ******************************************************************************/
bool LiDARHandler::CloseDB()
{
    // Acquire a write lock on the mutex to ensure thread safety.
    std::unique_lock<std::shared_mutex> lkWriteLock(m_muQueryMutex);

    // Handle the closing of the database and sqlite statement.
    if (m_bIsDBOpen)
    {
        // Finalize the prepared statement if it exists. This is necessary because
        // failing to do so can result in memory leaks. This just frees resources.
        if (m_pSQLStatement)
        {
            int nReturnCode = sqlite3_finalize(m_pSQLStatement);
            if (nReturnCode != SQLITE_OK)
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler: Failed to finalize SQL statement: {}", sqlite3_errmsg(m_pSQLDatabase));
                // Return false on failure.
                return false;
            }
            m_pSQLStatement = nullptr;    // Reset the statement pointer.
        }

        // Close the database connection.
        int nReturnCode = sqlite3_close(m_pSQLDatabase);
        if (nReturnCode != SQLITE_OK)
        {
            // Submit logger message.
            LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler: Failed to close database: {}", sqlite3_errmsg(m_pSQLDatabase));
            // Return false on failure.
            return false;
        }

        // Reset the database pointer and update the open flag.
        m_pSQLDatabase = nullptr;    // Reset the database pointer.
        m_bIsDBOpen    = false;      // Update the database open flag.
    }

    return true;
}

/******************************************************************************
 * @brief Retrieves LiDAR data points from the database based on the specified filter.
 *
 * @param stPointFilter - The filter criteria to apply when querying LiDAR data.
 * @return std::vector<LiDARHandler::PointRow> - A vector of PointRow structures containing the
 *         queried LiDAR data points.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-13
 ******************************************************************************/
std::vector<LiDARHandler::PointRow> LiDARHandler::GetLiDARData(const PointFilter& stPointFilter)
{
    // Acquire a read lock on the mutex to ensure thread safety.
    std::shared_lock<std::shared_mutex> lkReadLock(m_muQueryMutex);

    // Record the start time for performance measurement.
    std::chrono::time_point<std::chrono::high_resolution_clock> tmStartTime = std::chrono::high_resolution_clock::now();

    if (!m_bIsDBOpen)
    {
        LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler: Database is not open.");
        return {};
    }

    // Build dynamic WHERE clauses and binders.
    std::vector<std::string> vClauses;
    std::vector<std::function<void(sqlite3_stmt*, int&)>> vBinders;
    int nParamIndex = 1;

    // Spatial bounds are always present.
    vClauses.emplace_back("idx.min_x BETWEEN ? AND ?");
    vBinders.emplace_back(
        [&](sqlite3_stmt* sqlSTMT, int& nIndex)
        {
            sqlite3_bind_double(sqlSTMT, nIndex++, stPointFilter.dEasting - stPointFilter.dRadius);
            sqlite3_bind_double(sqlSTMT, nIndex++, stPointFilter.dEasting + stPointFilter.dRadius);
        });
    vClauses.emplace_back("idx.min_y BETWEEN ? AND ?");
    vBinders.emplace_back(
        [&](sqlite3_stmt* sqlSTMT, int& nIndex)
        {
            sqlite3_bind_double(sqlSTMT, nIndex++, stPointFilter.dNorthing - stPointFilter.dRadius);
            sqlite3_bind_double(sqlSTMT, nIndex++, stPointFilter.dNorthing + stPointFilter.dRadius);
        });

    // Optional classification.
    if (stPointFilter.szClassification && !stPointFilter.szClassification->empty())
    {
        vClauses.emplace_back("p.classification = ?");
        vBinders.emplace_back([&](sqlite3_stmt* sqlSTMT, int& nIndex)
                              { sqlite3_bind_text(sqlSTMT, nIndex++, stPointFilter.szClassification->c_str(), -1, SQLITE_STATIC); });
    }

    // Add optional filters.
    this->AddRangeFilter(vClauses, vBinders, "p.normal_x", stPointFilter.dNormalX);
    this->AddRangeFilter(vClauses, vBinders, "p.normal_y", stPointFilter.dNormalY);
    this->AddRangeFilter(vClauses, vBinders, "p.normal_z", stPointFilter.dNormalZ);
    this->AddRangeFilter(vClauses, vBinders, "p.slope", stPointFilter.dSlope);
    this->AddRangeFilter(vClauses, vBinders, "p.rough", stPointFilter.dRoughness);
    this->AddRangeFilter(vClauses, vBinders, "p.curvature", stPointFilter.dCurvature);
    this->AddRangeFilter(vClauses, vBinders, "p.trav_score", stPointFilter.dTraversalScore);

    // Construct final SQL query string.
    std::ostringstream stdOSS;
    stdOSS << "SELECT p.id, p.easting, p.northing, p.altitude, p.zone, p.classification,"
           << " p.normal_x, p.normal_y, p.normal_z, p.slope, p.rough, p.curvature, p.trav_score"
           << " FROM ProcessedLiDARPoints_idx AS idx"
           << " JOIN ProcessedLiDARPoints AS p ON p.id = idx.id"
           << " WHERE ";

    // Append all clauses to the SQL query.
    for (size_t siIter = 0; siIter < vClauses.size(); ++siIter)
    {
        if (siIter > 0)
            stdOSS << " AND ";
        stdOSS << vClauses[siIter];
    }

    // Final SQL query string.
    const std::string szSQLQuery = stdOSS.str();

    // Prepare SQL statement.
    sqlite3_stmt* sqlSTMT = nullptr;
    int nRC               = sqlite3_prepare_v2(m_pSQLDatabase, szSQLQuery.c_str(), -1, &sqlSTMT, nullptr);
    if (nRC != SQLITE_OK)
    {
        LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler: Failed to prepare SQL: {}", sqlite3_errmsg(m_pSQLDatabase));
        return {};
    }

    // Bind parameters.
    for (std::function<void(sqlite3_stmt*, int&)>& binder : vBinders)
    {
        binder(sqlSTMT, nParamIndex);
    }

    // Execute and collect results.
    std::vector<PointRow> vResults;
    while ((nRC = sqlite3_step(sqlSTMT)) == SQLITE_ROW)
    {
        PointRow stRow;
        stRow.nID              = sqlite3_column_int(sqlSTMT, 0);
        stRow.dEasting         = sqlite3_column_double(sqlSTMT, 1);
        stRow.dNorthing        = sqlite3_column_double(sqlSTMT, 2);
        stRow.dAltitude        = sqlite3_column_double(sqlSTMT, 3);
        stRow.szZone           = reinterpret_cast<const char*>(sqlite3_column_text(sqlSTMT, 4));
        stRow.szClassification = reinterpret_cast<const char*>(sqlite3_column_text(sqlSTMT, 5));
        stRow.dNormalX         = sqlite3_column_double(sqlSTMT, 6);
        stRow.dNormalY         = sqlite3_column_double(sqlSTMT, 7);
        stRow.dNormalZ         = sqlite3_column_double(sqlSTMT, 8);
        stRow.dSlope           = sqlite3_column_double(sqlSTMT, 9);
        stRow.dRoughness       = sqlite3_column_double(sqlSTMT, 10);
        stRow.dCurvature       = sqlite3_column_double(sqlSTMT, 11);
        stRow.dTraversalScore  = sqlite3_column_double(sqlSTMT, 12);
        vResults.push_back(stRow);
    }

    // Finalize SQL statement.
    if ((nRC = sqlite3_finalize(sqlSTMT)) != SQLITE_OK)
    {
        // Submit logger message.
        LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler: Failed to finalize statement: {}", sqlite3_errmsg(m_pSQLDatabase));
        // Return empty results on failure.
        return {};
    }

    // Record the end time for performance measurement.
    std::chrono::time_point<std::chrono::high_resolution_clock> tmEndTime = std::chrono::high_resolution_clock::now();
    double dQueryTime                                                     = std::chrono::duration<double>(tmEndTime - tmStartTime).count();

    // If time is over 1 second log a warning.
    if (dQueryTime > 1.0)
    {
        // Submit logger message.
        LOG_WARNING(logging::g_qSharedLogger, "LiDARHandler: Query took {:.2f} seconds to execute.", dQueryTime);
    }
    else
    {
        LOG_DEBUG(logging::g_qSharedLogger, "LiDARHandler: Query took {} seconds to execute.", dQueryTime);
    }

    return vResults;
}

/******************************************************************************
 * @brief Inserts LiDAR data points into the database.
 *
 * @param vPoints - Vector of geoops::Waypoint structures containing the data points to insert.
 * @return true - If the data points were successfully inserted.
 * @return false - If the insertion failed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-10-20
 ******************************************************************************/
bool LiDARHandler::InsertLiDARData(const std::vector<geoops::Waypoint>& vPoints)
{
    // Acquire a write lock on the mutex to ensure thread safety.
    std::unique_lock<std::shared_mutex> lkWriteLock(m_muQueryMutex);

    // Check if the database is open.
    if (!m_bIsDBOpen)
    {
        LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler: Database is not open.");
        return false;
    }

    // Prepare the SQL statement for inserting data.
    const char* pSQL      = R"(
        INSERT INTO ProcessedLiDARPoints (easting, northing, altitude, zone, classification, normal_x, normal_y, normal_z, slope, rough, curvature, trav_score)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    )";

    sqlite3_stmt* sqlSTMT = nullptr;
    int nRC               = sqlite3_prepare_v2(m_pSQLDatabase, pSQL, -1, &sqlSTMT, nullptr);
    if (nRC != SQLITE_OK)
    {
        LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler: Failed to prepare SQL: {}", sqlite3_errmsg(m_pSQLDatabase));
        return false;
    }

    // Process the input waypoints into PointRow structures.
    std::vector<PointRow> vProcessedPoints;
    for (const geoops::Waypoint& stWaypoint : vPoints)
    {
        PointRow point;
        point.dEasting         = stWaypoint.GetUTMCoordinate().dEasting;
        point.dNorthing        = stWaypoint.GetUTMCoordinate().dNorthing;
        point.dAltitude        = stWaypoint.GetUTMCoordinate().dAltitude;
        point.szZone           = std::to_string(stWaypoint.GetUTMCoordinate().nZone) + (stWaypoint.GetUTMCoordinate().bWithinNorthernHemisphere ? "N" : "S");
        point.szClassification = "unknown";    // Default classification; modify as needed.
        point.dNormalX         = 0.0;          // Placeholder; modify as needed.
        point.dNormalY         = 0.0;          // Placeholder; modify as needed.
        point.dNormalZ         = 0.0;          // Placeholder; modify as needed.
        point.dSlope           = 0.0;          // Placeholder; modify as needed.
        point.dRoughness       = 0.0;          // Placeholder; modify as needed.
        point.dCurvature       = 0.0;          // Placeholder; modify as needed.
        point.dTraversalScore  = 0.0;          // Placeholder; modify as needed.
        vProcessedPoints.push_back(point);
    }

    // Bind parameters for each point.
    for (const PointRow& point : vProcessedPoints)
    {
        sqlite3_bind_double(sqlSTMT, 1, point.dEasting);
        sqlite3_bind_double(sqlSTMT, 2, point.dNorthing);
        sqlite3_bind_double(sqlSTMT, 3, point.dAltitude);
        sqlite3_bind_text(sqlSTMT, 4, point.szZone.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_text(sqlSTMT, 5, point.szClassification.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_double(sqlSTMT, 6, point.dNormalX);
        sqlite3_bind_double(sqlSTMT, 7, point.dNormalY);
        sqlite3_bind_double(sqlSTMT, 8, point.dNormalZ);
        sqlite3_bind_double(sqlSTMT, 9, point.dSlope);
        sqlite3_bind_double(sqlSTMT, 10, point.dRoughness);
        sqlite3_bind_double(sqlSTMT, 11, point.dCurvature);
        sqlite3_bind_double(sqlSTMT, 12, point.dTraversalScore);

        // Execute the statement.
        nRC = sqlite3_step(sqlSTMT);
        if (nRC != SQLITE_DONE)
        {
            LOG_ERROR(logging::g_qSharedLogger, "LiDARHandler: Failed to insert data: {}", sqlite3_errmsg(m_pSQLDatabase));
            sqlite3_finalize(sqlSTMT);
            return false;
        }

        // Reset the statement for the next iteration.
        sqlite3_reset(sqlSTMT);
    }

    // Finalize the statement.
    sqlite3_finalize(sqlSTMT);
    return true;
}

/******************************************************************************
 * @brief Checks if the database is currently open.
 *
 * @return true - If the database is open.
 * @return false - If the database is not open.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
bool LiDARHandler::IsDBOpen()
{
    // Acquire a read lock on the mutex to ensure thread safety.
    std::shared_lock<std::shared_mutex> lkReadLock(m_muQueryMutex);
    // Return the database open status.
    return m_bIsDBOpen;
}

/******************************************************************************
 * @brief Adds a range filter to the SQL query clauses and binders.
 *
 * @tparam T - The data type of the range values.
 * @param vClauses - The vector of SQL clauses to which the range filter will be added.
 * @param vBinders - The vector of binders for the SQL statement.
 * @param pColumn - The name of the column to apply the range filter on.
 * @param stdOptRange - The optional range to filter by. If it is not set, no filter will be added.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
template<typename T>
void LiDARHandler::AddRangeFilter(std::vector<std::string>& vClauses,
                                  std::vector<std::function<void(sqlite3_stmt*, int&)>>& vBinders,
                                  const char* pColumn,
                                  const std::optional<PointFilter::Range<T>>& stdOptRange)
{
    // If the range is set, add the filter clause and binder.
    if (stdOptRange)
    {
        // Add the range filter clause based on the type of T.
        if constexpr (std::is_floating_point<T>::value)
        {
            // Only use >= by default for floats
            vClauses.emplace_back(std::string(pColumn) + " >= ?");
            vBinders.emplace_back([&](sqlite3_stmt* sqlSTMT, int& nIndex) { sqlite3_bind_double(sqlSTMT, nIndex++, stdOptRange->tMin); });
        }
        else
        {
            // Use BETWEEN for ints or guaranteed bounded ranges
            vClauses.emplace_back(std::string(pColumn) + " BETWEEN ? AND ?");
            vBinders.emplace_back(
                [&](sqlite3_stmt* sqlSTMT, int& nIndex)
                {
                    sqlite3_bind_double(sqlSTMT, nIndex++, stdOptRange->tMin);
                    sqlite3_bind_double(sqlSTMT, nIndex++, stdOptRange->tMax);
                });
        }
    }
}
