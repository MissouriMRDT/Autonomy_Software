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
 *       via Quill Logger and this function will return false.
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
        LOG_WARNING(logging::g_qSharedLogger, "Database is already open. Closing existing connection before opening a new one.");
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
        LOG_ERROR(logging::g_qSharedLogger, "Failed to open database at '{}': {}", szDBPath, sqlite3_errmsg(m_pSQLDatabase));
        // Return false on failure.
        return false;
    }

    // Set the database open flag to true.
    m_bIsDBOpen = true;

    // Log success.
    LOG_INFO(logging::g_qSharedLogger, "Successfully opened database at '{}'.", szDBPath);

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
                LOG_ERROR(logging::g_qSharedLogger, "Failed to finalize SQL statement: {}", sqlite3_errmsg(m_pSQLDatabase));
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
            LOG_ERROR(logging::g_qSharedLogger, "Failed to close database: {}", sqlite3_errmsg(m_pSQLDatabase));
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
        LOG_ERROR(logging::g_qSharedLogger, "Database is not open.");
        return {};
    }

    // Build dynamic WHERE clauses and binders.
    std::vector<std::string> vClauses;
    std::vector<std::function<void(sqlite3_stmt*, int&)>> vBinders;
    int nParamIndex = 1;

    // Spatial bounds are always present (using R-Tree index).
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

    // Optional classification filter.
    // OPTIMIZATION: We filter against the joined 'Classifications' table (c.label).
    if (stPointFilter.szClassification && !stPointFilter.szClassification->empty())
    {
        vClauses.emplace_back("c.label = ?");
        vBinders.emplace_back([&](sqlite3_stmt* sqlSTMT, int& nIndex)
                              { sqlite3_bind_text(sqlSTMT, nIndex++, stPointFilter.szClassification->c_str(), -1, SQLITE_STATIC); });
    }

    // Add optional filters for metrics.
    this->AddRangeFilter(vClauses, vBinders, "p.normal_x", stPointFilter.dNormalX);
    this->AddRangeFilter(vClauses, vBinders, "p.normal_y", stPointFilter.dNormalY);
    this->AddRangeFilter(vClauses, vBinders, "p.normal_z", stPointFilter.dNormalZ);
    this->AddRangeFilter(vClauses, vBinders, "p.slope", stPointFilter.dSlope);
    this->AddRangeFilter(vClauses, vBinders, "p.rough", stPointFilter.dRoughness);
    this->AddRangeFilter(vClauses, vBinders, "p.curvature", stPointFilter.dCurvature);
    this->AddRangeFilter(vClauses, vBinders, "p.trav_score", stPointFilter.dTraversalScore);

    // Construct final SQL query string.
    std::ostringstream stdOSS;

    // OPTIMIZATION:
    // 1. Select 'z.label' and 'c.label' to get human-readable text.
    // 2. LEFT JOIN to handle cases where IDs might not map (prevents data loss).
    stdOSS << "SELECT p.id, p.easting, p.northing, p.altitude, z.label, c.label,"
           << " p.normal_x, p.normal_y, p.normal_z, p.slope, p.rough, p.curvature, p.trav_score"
           << " FROM ProcessedLiDARPoints_idx AS idx"
           << " JOIN ProcessedLiDARPoints AS p ON p.id = idx.id"
           << " LEFT JOIN Zones AS z ON p.zone_id = z.id"
           << " LEFT JOIN Classifications AS c ON p.class_code = c.code"
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
        LOG_ERROR(logging::g_qSharedLogger, "Failed to prepare SQL: {}", sqlite3_errmsg(m_pSQLDatabase));
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
        stRow.nID       = sqlite3_column_int(sqlSTMT, 0);
        stRow.dEasting  = sqlite3_column_double(sqlSTMT, 1);
        stRow.dNorthing = sqlite3_column_double(sqlSTMT, 2);
        stRow.dAltitude = sqlite3_column_double(sqlSTMT, 3);

        // --- SEGFAULT FIX START ---
        // Retrieve Zone (Column 4). Check for NULL (if LEFT JOIN failed).
        const char* pszZone = reinterpret_cast<const char*>(sqlite3_column_text(sqlSTMT, 4));
        stRow.szZone        = pszZone ? pszZone : "Unknown";

        // Retrieve Classification (Column 5). Check for NULL.
        const char* pszClass   = reinterpret_cast<const char*>(sqlite3_column_text(sqlSTMT, 5));
        stRow.szClassification = pszClass ? pszClass : "Unclassified";
        // --- SEGFAULT FIX END ---

        stRow.dNormalX        = sqlite3_column_double(sqlSTMT, 6);
        stRow.dNormalY        = sqlite3_column_double(sqlSTMT, 7);
        stRow.dNormalZ        = sqlite3_column_double(sqlSTMT, 8);
        stRow.dSlope          = sqlite3_column_double(sqlSTMT, 9);
        stRow.dRoughness      = sqlite3_column_double(sqlSTMT, 10);
        stRow.dCurvature      = sqlite3_column_double(sqlSTMT, 11);
        stRow.dTraversalScore = sqlite3_column_double(sqlSTMT, 12);

        vResults.push_back(stRow);
    }

    // Finalize SQL statement.
    if ((nRC = sqlite3_finalize(sqlSTMT)) != SQLITE_OK)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Failed to finalize statement: {}", sqlite3_errmsg(m_pSQLDatabase));
        return {};
    }

    // Record the end time for performance measurement.
    std::chrono::time_point<std::chrono::high_resolution_clock> tmEndTime = std::chrono::high_resolution_clock::now();
    double dQueryTime                                                     = std::chrono::duration<double>(tmEndTime - tmStartTime).count();

    if (dQueryTime > 0.2)
    {
        LOG_WARNING(logging::g_qSharedLogger, "Query took {:.2f} seconds to execute.", dQueryTime);
    }
    else
    {
        LOG_DEBUG(logging::g_qSharedLogger, "Query took {} seconds to execute.", dQueryTime);
    }

    if (vResults.empty())
    {
        LOG_WARNING(logging::g_qSharedLogger, "Query returned no results.");
    }

    return vResults;
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

/******************************************************************************
 * @brief Modifies all LiDAR points in radius to reflect bad terrain
 *
 * @param stPoint - Center UTM coordinate of obstacle
 * @param dRadius - Radius of obstacle
 * @return true - If the data points were successfully modified.
 * @return false - If the modification failed.
 *
 * @author clayjay3 (claytonraycowen@gmail.com), Sam Nolte (samnolte0302@gmail.com)
 * @date 2025-1-12
 ******************************************************************************/
bool LiDARHandler::DeclareLiDARObstacle(geoops::UTMCoordinate stPoint, double dRadius)
{
    // Acquire a write lock on the mutex to ensure thread safety.
    std::unique_lock<std::shared_mutex> lkWriteLock(m_muQueryMutex);

    // Check if the database is open.
    if (!m_bIsDBOpen)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Database is not open.");
        return false;
    }

    // Prepare the SQL statements for inserting data.
    // TODO: Experiment with the more circle-like combination of shapes vs execution time
    const char* pSQL            = R"(
        UPDATE ProcessedLiDARPoints
        SET trav-score = 0.0
        WHERE
        (
            easting > ?
            AND easting < ?
            AND northing > ?
            AND northing < ?
        )
        OR
        (
            easting > ?
            AND easting < ?
            AND northing > ?
            AND northing < ?
        )
    )";

    sqlite3_stmt* sqlDeleteSTMT = nullptr;
    int nRC                     = sqlite3_prepare_v2(m_pSQLDatabase, pSQL, -1, &sqlDeleteSTMT, nullptr);
    if (nRC != SQLITE_OK)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Failed to prepare SQL: {}", sqlite3_errmsg(m_pSQLDatabase));
        return false;
    }

    double dH = dRadius * sin(2.0 / 6.0 * M_PI);
    double dK = dRadius * cos(2.0 / 6.0 * M_PI);

    sqlite3_bind_double(sqlDeleteSTMT, 1, stPoint.dEasting - dK);
    sqlite3_bind_double(sqlDeleteSTMT, 2, stPoint.dEasting + dK);
    sqlite3_bind_double(sqlDeleteSTMT, 3, stPoint.dNorthing - dH);
    sqlite3_bind_double(sqlDeleteSTMT, 4, stPoint.dNorthing + dH);
    sqlite3_bind_double(sqlDeleteSTMT, 5, stPoint.dEasting - dH);
    sqlite3_bind_double(sqlDeleteSTMT, 6, stPoint.dEasting + dH);
    sqlite3_bind_double(sqlDeleteSTMT, 7, stPoint.dNorthing - dK);
    sqlite3_bind_double(sqlDeleteSTMT, 8, stPoint.dNorthing + dK);

    // Execute the statement.
    nRC = sqlite3_step(sqlDeleteSTMT);
    if (nRC != SQLITE_DONE)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Failed to insert data: {}", sqlite3_errmsg(m_pSQLDatabase));
        sqlite3_finalize(sqlDeleteSTMT);
        return false;
    }

    // Finalize the statement.
    sqlite3_finalize(sqlDeleteSTMT);

    // // Prepare the SQL statements for inserting data.
    // const char* pSQL            = R"(
    //     INSERT INTO ProcessedLiDARPoints (easting, northing, altitude, zone, classification, normal_x, normal_y, normal_z, slope, rough, curvature, trav_score)
    //     VALUES (?, ?, ?, ?, ?, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    // )";

    // sqlite3_stmt* sqlInsertSTMT = nullptr;
    // int nRC                     = sqlite3_prepare_v2(m_pSQLDatabase, pSQL, -1, &sqlInsertSTMT, nullptr);
    // if (nRC != SQLITE_OK)
    // {
    //     LOG_ERROR(logging::g_qSharedLogger, "Failed to prepare SQL: {}", sqlite3_errmsg(m_pSQLDatabase));
    //     return false;
    // }

    // // Add a couple points around the circle
    // double dPercentDist   = 0.90;
    // double dOffsets[9][2] = {
    //     {-dPercentDist / sqrt(2), dPercentDist / sqrt(2)},     // top-left
    //     {0, dPercentDist},                                     // top-middle
    //     {dPercentDist / sqrt(2), dPercentDist / sqrt(2)},      // top-right
    //     {-dPercentDist, 0},                                    // middle-left
    //     {0, 0},                                                // middle-middle
    //     {dPercentDist, 0},                                     // middle-right
    //     {-dPercentDist / sqrt(2), -dPercentDist / sqrt(2)},    // bottom-left
    //     {0, -dPercentDist},                                    // bottom-middle
    //     {dPercentDist / sqrt(2), -dPercentDist / sqrt(2)}      // middle-right
    // };

    // // Process the input waypoints into PointRow structures.
    // std::vector<PointRow> vProcessedPoints;
    // for (int i = 0; i < 9; ++i)
    // {
    //     PointRow point;
    //     point.dEasting         = stWaypoint.GetUTMCoordinate().dEasting + dRadius * dOffset[i][0];
    //     point.dNorthing        = stWaypoint.GetUTMCoordinate().dNorthing + dRadius * dOffset[i][1];
    //     point.dAltitude        = stWaypoint.GetUTMCoordinate().dAltitude;
    //     point.szZone           = std::to_string(stWaypoint.GetUTMCoordinate().nZone) + (stWaypoint.GetUTMCoordinate().bWithinNorthernHemisphere ? "N" : "S");
    //     point.szClassification = "obstacle";
    //     vProcessedPoints.push_back(point);
    // }

    // // Bind parameters for each point.
    // for (const PointRow& point : vProcessedPoints)
    // {
    //     sqlite3_bind_double(sqlInsertSTMT, 1, point.dEasting);
    //     sqlite3_bind_double(sqlInsertSTMT, 2, point.dNorthing);
    //     sqlite3_bind_double(sqlInsertSTMT, 3, point.dAltitude);
    //     sqlite3_bind_text(sqlInsertSTMT, 4, point.szZone.c_str(), -1, SQLITE_STATIC);
    //     sqlite3_bind_text(sqlInsertSTMT, 5, point.szClassification.c_str(), -1, SQLITE_STATIC);

    //     // Execute the statement.
    //     nRC = sqlite3_step(sqlInsertSTMT);
    //     if (nRC != SQLITE_DONE)
    //     {
    //         LOG_ERROR(logging::g_qSharedLogger, "Failed to insert data: {}", sqlite3_errmsg(m_pSQLDatabase));
    //         sqlite3_finalize(sqlInsertSTMT);
    //         return false;
    //     }

    //     // Reset the statement for the next iteration.
    //     sqlite3_reset(sqlInsertSTMT);
    // }

    // // Finalize the statement.
    // sqlite3_finalize(sqlInsertSTMT);
    return true;
}
