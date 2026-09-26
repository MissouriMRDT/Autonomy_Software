/******************************************************************************
 * @brief Implementation of the LiDAR runtime query interface using DuckDB.
 *
 * @file LiDARHandler.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com), Eli Byrd (edbgkk@mst.edu)
 * @date 2025-05-20
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "LiDARHandler.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/// \cond
#include <filesystem>

/// \endcond

/******************************************************************************
 * @brief Construct a new LiDARHandler::LiDARHandler object.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-07-13
 ******************************************************************************/
LiDARHandler::LiDARHandler()
{
    // Ensure smart pointers are null natively.
    m_pDB       = nullptr;
    m_bIsDBOpen = false;
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
 * @brief Initializes the LiDARHandler by opening the DuckDB database.
 *
 * This method securely opens the DuckDB file and instantiates a persistent
 * connection object. The connection is opened in default Read/Write mode
 * to allow the autonomy system to dynamically modify the terrain and declare
 * obstacles at runtime.
 *
 * @param szDBPath Relative or absolute path to the DuckDB database file.
 *
 * @return true - If the database was successfully opened.
 * @return false - If there was an error opening the database.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2026-05-01
 ******************************************************************************/
bool LiDARHandler::OpenDB(const std::string& szDBPath)
{
    // Check if the file actually exists on disk before doing anything.
    if (!std::filesystem::exists(szDBPath))
    {
        LOG_ERROR(logging::g_qSharedLogger, "Failed to open DuckDB: File does not exist at '{}'", szDBPath);
        return false;
    }

    // Acquire a write lock on the mutex to ensure thread safety.
    std::unique_lock lkWriteLock(m_muQueryMutex);

    // Reset existing connection if already open (fixing the previous race condition)
    if (m_bIsDBOpen)
    {
        LOG_WARNING(logging::g_qSharedLogger, "Database is already open. Closing existing connection before opening a new one.");
        m_pDB.reset();
        m_bIsDBOpen = false;
    }

    try
    {
        // Instantiate the DuckDB instance.
        m_pDB = std::make_unique<duckdb::DuckDB>(szDBPath);
    }
    catch (const duckdb::Exception& e)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Failed to open DuckDB at '{}': {}", szDBPath, e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Standard exception while opening DuckDB: {}", e.what());
        return false;
    }

    m_bIsDBOpen = true;
    LOG_INFO(logging::g_qSharedLogger, "Successfully opened DuckDB analytics engine at '{}'.", szDBPath);

    return true;
}

/******************************************************************************
 * @brief Closes the currently open LiDAR DuckDB connection.
 *
 * @return true - If the database was successfully closed.
 * @return false - If there was an error closing the database.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-07-13
 ******************************************************************************/
bool LiDARHandler::CloseDB()
{
    std::unique_lock lkWriteLock(m_muQueryMutex);

    if (m_bIsDBOpen)
    {
        // Smart pointers automatically release resources and close database locks
        // when reset. This avoids SQLite's manual finalize() memory leak issues.
        m_pDB.reset();
        m_bIsDBOpen = false;
    }

    return true;
}

/******************************************************************************
 * @brief Retrieves LiDAR data points from DuckDB based on the specified filter.
 *
 * @param stPointFilter - The filter criteria to apply when querying LiDAR data.
 * @return std::vector<LiDARHandler::PointRow> - Queried rows from the database.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-07-13
 ******************************************************************************/
std::vector<LiDARHandler::PointRow> LiDARHandler::GetLiDARData(const PointFilter& stPointFilter)
{
    ZoneScopedC(tracy::Color::SlateGray1);
    std::shared_lock lkReadLock(m_muQueryMutex);
    std::chrono::time_point<std::chrono::high_resolution_clock> tmStartTime = std::chrono::high_resolution_clock::now();

    if (!m_bIsDBOpen)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Database is not open.");
        return {};
    }

    // Build dynamic WHERE clauses. DuckDB binds values natively into vectors.
    std::vector<std::string> vClauses;
    duckdb::vector<duckdb::Value> vBindValues;

    // Spatial bounds are applied directly to the columns. DuckDB uses underlying
    // block Zonemaps to instantly skip unneeded file blocks on disk.
    vClauses.emplace_back("p.easting BETWEEN ? AND ?");
    vBindValues.push_back(duckdb::Value(stPointFilter.dEasting - stPointFilter.dRadius));
    vBindValues.push_back(duckdb::Value(stPointFilter.dEasting + stPointFilter.dRadius));

    vClauses.emplace_back("p.northing BETWEEN ? AND ?");
    vBindValues.push_back(duckdb::Value(stPointFilter.dNorthing - stPointFilter.dRadius));
    vBindValues.push_back(duckdb::Value(stPointFilter.dNorthing + stPointFilter.dRadius));

    // Optional classification filter
    if (stPointFilter.szClassification && !stPointFilter.szClassification->empty())
    {
        vClauses.emplace_back("c.label = ?");
        vBindValues.push_back(duckdb::Value(*stPointFilter.szClassification));
    }

    // Add optional filters for metrics using COALESCE to safely treat NULL edge-points as 0.0.
    this->AddRangeFilter(vClauses, vBindValues, "COALESCE(p.normal_x, 0.0)", stPointFilter.dNormalX);
    this->AddRangeFilter(vClauses, vBindValues, "COALESCE(p.normal_y, 0.0)", stPointFilter.dNormalY);
    this->AddRangeFilter(vClauses, vBindValues, "COALESCE(p.normal_z, 0.0)", stPointFilter.dNormalZ);
    this->AddRangeFilter(vClauses, vBindValues, "COALESCE(p.slope, 0.0)", stPointFilter.dSlope);
    this->AddRangeFilter(vClauses, vBindValues, "COALESCE(p.rough, 0.0)", stPointFilter.dRoughness);
    this->AddRangeFilter(vClauses, vBindValues, "COALESCE(p.curvature, 0.0)", stPointFilter.dCurvature);
    this->AddRangeFilter(vClauses, vBindValues, "COALESCE(p.trav_score, 0.0)", stPointFilter.dTraversalScore);

    // Construct final SQL query string.
    std::ostringstream stdOSS;
    // Every column is cast to the exact type the result loop reads it as, so the loop can read the raw column arrays.
    stdOSS << "SELECT CAST(p.id AS INTEGER), CAST(p.easting AS DOUBLE), CAST(p.northing AS DOUBLE), CAST(p.altitude AS DOUBLE),"
           << " CAST(z.label AS VARCHAR), CAST(c.label AS VARCHAR),"
           << " CAST(COALESCE(p.normal_x, 0.0) AS DOUBLE), CAST(COALESCE(p.normal_y, 0.0) AS DOUBLE), CAST(COALESCE(p.normal_z, 0.0) AS DOUBLE),"
           << " CAST(COALESCE(p.slope, 0.0) AS DOUBLE), CAST(COALESCE(p.rough, 0.0) AS DOUBLE), CAST(COALESCE(p.curvature, 0.0) AS DOUBLE),"
           << " CAST(COALESCE(p.trav_score, 0.0) AS DOUBLE)"
           << " FROM ProcessedLiDARPoints AS p"
           << " LEFT JOIN Zones AS z ON p.zone_id = z.id"
           << " LEFT JOIN Classifications AS c ON p.class_code = c.code"
           << " WHERE ";

    for (size_t siIter = 0; siIter < vClauses.size(); ++siIter)
    {
        if (siIter > 0)
            stdOSS << " AND ";
        stdOSS << vClauses[siIter];
    }

    std::vector<PointRow> vResults;

    try
    {
        // Thread-local connection to avoid stepping on pending chunk states
        duckdb::Connection stLocalConn(*m_pDB);

        // DuckDB Prepared Statements protect against injections and compile the plan
        duckdb::unique_ptr<duckdb::PreparedStatement> stPreparedStmt = stLocalConn.Prepare(stdOSS.str());
        if (stPreparedStmt->HasError())
        {
            LOG_ERROR(logging::g_qSharedLogger, "Failed to prepare DuckDB SQL: {}", stPreparedStmt->GetError());
            return {};
        }

        // Execute the query passing the bound values
        duckdb::unique_ptr<duckdb::QueryResult> stResult = stPreparedStmt->Execute(vBindValues);
        if (stResult->HasError())
        {
            LOG_ERROR(logging::g_qSharedLogger, "Execution Error: {}", stResult->GetError());
            return {};
        }

        // DuckDB extracts data in vector chunks. Iterating via Fetch() is extremely performant
        // and naturally manages memory without locking threads.
        while (duckdb::unique_ptr<duckdb::DataChunk> stChunk = stResult->Fetch())
        {
            // Read each column as a plain typed array. Fetching cells one at a time with GetValue() boxed every one of
            // the 13 cells per row in a duckdb::Value, which cost far more than the query itself on large tiles.
            // Flatten() turns constant and dictionary columns into plain arrays so they can be read the same way.
            stChunk->Flatten();
            const size_t siRows                = stChunk->size();
            const int32_t* pIDs                = duckdb::FlatVector::GetData<int32_t>(stChunk->data[0]);
            const double* pEastings            = duckdb::FlatVector::GetData<double>(stChunk->data[1]);
            const double* pNorthings           = duckdb::FlatVector::GetData<double>(stChunk->data[2]);
            const double* pAltitudes           = duckdb::FlatVector::GetData<double>(stChunk->data[3]);
            const duckdb::string_t* pZones     = duckdb::FlatVector::GetData<duckdb::string_t>(stChunk->data[4]);
            const duckdb::string_t* pClasses   = duckdb::FlatVector::GetData<duckdb::string_t>(stChunk->data[5]);
            const double* pNormalXs            = duckdb::FlatVector::GetData<double>(stChunk->data[6]);
            const double* pNormalYs            = duckdb::FlatVector::GetData<double>(stChunk->data[7]);
            const double* pNormalZs            = duckdb::FlatVector::GetData<double>(stChunk->data[8]);
            const double* pSlopes              = duckdb::FlatVector::GetData<double>(stChunk->data[9]);
            const double* pRoughnesses         = duckdb::FlatVector::GetData<double>(stChunk->data[10]);
            const double* pCurvatures          = duckdb::FlatVector::GetData<double>(stChunk->data[11]);
            const double* pTraversalScores     = duckdb::FlatVector::GetData<double>(stChunk->data[12]);
            duckdb::ValidityMask& stAltValid   = duckdb::FlatVector::Validity(stChunk->data[3]);
            duckdb::ValidityMask& stZoneValid  = duckdb::FlatVector::Validity(stChunk->data[4]);
            duckdb::ValidityMask& stClassValid = duckdb::FlatVector::Validity(stChunk->data[5]);

            vResults.reserve(vResults.size() + siRows);
            for (size_t siIter = 0; siIter < siRows; siIter++)
            {
                PointRow stRow;

                // Extract native datatypes directly from the memory chunk.
                stRow.nID              = pIDs[siIter];
                stRow.dEasting         = pEastings[siIter];
                stRow.dNorthing        = pNorthings[siIter];
                stRow.dAltitude        = stAltValid.RowIsValid(siIter) ? pAltitudes[siIter] : 0.0;
                stRow.szZone           = stZoneValid.RowIsValid(siIter) ? pZones[siIter].GetString() : "Unknown";
                stRow.szClassification = stClassValid.RowIsValid(siIter) ? pClasses[siIter].GetString() : "Unclassified";

                // Metrics are guaranteed to be non-null due to the COALESCE in the SELECT clause.
                stRow.dNormalX        = pNormalXs[siIter];
                stRow.dNormalY        = pNormalYs[siIter];
                stRow.dNormalZ        = pNormalZs[siIter];
                stRow.dSlope          = pSlopes[siIter];
                stRow.dRoughness      = pRoughnesses[siIter];
                stRow.dCurvature      = pCurvatures[siIter];
                stRow.dTraversalScore = pTraversalScores[siIter];

                vResults.push_back(std::move(stRow));
            }
        }
    }
    catch (const duckdb::Exception& e)
    {
        LOG_ERROR(logging::g_qSharedLogger, "DuckDB threw an exception in GetLiDARData: {}", e.what());
        return {};
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(logging::g_qSharedLogger, "A standard exception was thrown in GetLiDARData: {}", e.what());
        return {};
    }

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
 * @return true - If the database is open.
 * @return false - If the database is not open.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
bool LiDARHandler::IsDBOpen()
{
    std::shared_lock lkReadLock(m_muQueryMutex);
    return m_bIsDBOpen;
}

/******************************************************************************
 * @brief Adds a range filter to the SQL query clauses and dynamically bound values.
 *
 * @tparam T - The data type of the range values.
 * @param vClauses - The vector of SQL clauses to which the range filter will be added.
 * @param vBindValues - The duckdb value container storing runtime query parameters.
 * @param pColumn - The name of the column to apply the range filter on.
 * @param stdOptRange - The optional range to filter by.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2025-07-14
 ******************************************************************************/
template<typename T>
void LiDARHandler::AddRangeFilter(std::vector<std::string>& vClauses,
                                  duckdb::vector<duckdb::Value>& vBindValues,
                                  const char* pColumn,
                                  const std::optional<PointFilter::Range<T>>& stdOptRange)
{
    if (stdOptRange)
    {
        vClauses.emplace_back(std::string(pColumn) + " BETWEEN ? AND ?");
        vBindValues.push_back(duckdb::Value(stdOptRange->tMin));
        vBindValues.push_back(duckdb::Value(stdOptRange->tMax));
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
 * @date 2026-05-01
 ******************************************************************************/
bool LiDARHandler::DeclareLiDARObstacle(const geoops::UTMCoordinate& stPoint, double dRadius)
{
    ZoneScopedC(tracy::Color::SlateGray1);

    // Acquire a write lock on the mutex to ensure thread safety.
    std::unique_lock lkWriteLock(m_muQueryMutex);

    // Check if the database is open.
    if (!m_bIsDBOpen)
    {
        LOG_ERROR(logging::g_qSharedLogger, "Database is not open.");
        return false;
    }

    // Prepare the SQL statement for updating data.
    // DuckDB zonemaps replace the need for the old RTree index table.
    const char* pSQL = R"(
        UPDATE ProcessedLiDARPoints
        SET trav_score = 0.01
        WHERE easting BETWEEN ? AND ?
          AND northing BETWEEN ? AND ?
          AND (easting - ?) * (easting - ?) + (northing - ?) * (northing - ?) <= ?
    )";

    // Bind values to the prepared statement
    duckdb::vector<duckdb::Value> vBindValues;

    // Bounding box (zonemap fast-filter)
    vBindValues.push_back(duckdb::Value(stPoint.dEasting - dRadius));
    vBindValues.push_back(duckdb::Value(stPoint.dEasting + dRadius));
    vBindValues.push_back(duckdb::Value(stPoint.dNorthing - dRadius));
    vBindValues.push_back(duckdb::Value(stPoint.dNorthing + dRadius));

    // Radial distance check (exact filtering)
    vBindValues.push_back(duckdb::Value(stPoint.dEasting));
    vBindValues.push_back(duckdb::Value(stPoint.dEasting));
    vBindValues.push_back(duckdb::Value(stPoint.dNorthing));
    vBindValues.push_back(duckdb::Value(stPoint.dNorthing));

    // Pass the squared radius directly so we don't need ? * ? in the SQL
    vBindValues.push_back(duckdb::Value(dRadius * dRadius));

    int64_t nRowsUpdated = 0;
    try
    {
        // Thread-local connection to safely issue commands without bleeding pending states
        duckdb::Connection stLocalConn(*m_pDB);

        duckdb::unique_ptr<duckdb::PreparedStatement> stPreparedStmt = stLocalConn.Prepare(pSQL);
        if (stPreparedStmt->HasError())
        {
            LOG_ERROR(logging::g_qSharedLogger, "Failed to prepare DuckDB SQL: {}", stPreparedStmt->GetError());
            return false;
        }

        // Execute the statement.
        auto stResult = stPreparedStmt->Execute(vBindValues);
        if (stResult->HasError())
        {
            LOG_ERROR(logging::g_qSharedLogger, "Failed to update obstacle data: {}", stResult->GetError());
            return false;
        }

        // DuckDB UPDATE queries return a single column/row chunk containing the number of updated rows.
        // It's critical to loop over Fetch() until it returns null to exhaust the result set.
        while (auto stChunk = stResult->Fetch())
        {
            if (stChunk->size() > 0 && nRowsUpdated == 0)
            {
                nRowsUpdated = stChunk->GetValue(0, 0).GetValue<int64_t>();
            }
        }
    }
    catch (const duckdb::Exception& e)
    {
        LOG_ERROR(logging::g_qSharedLogger, "DuckDB threw an exception in DeclareLiDARObstacle: {}", e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(logging::g_qSharedLogger, "A standard exception was thrown in DeclareLiDARObstacle: {}", e.what());
        return false;
    }

    // Log LiDAR changes
    LOG_INFO(logging::g_qSharedLogger,
             "Created new obstacle at ({}, {}), radius: {}. Updated {} points",
             (int) stPoint.dEasting,
             (int) stPoint.dNorthing,
             (int) dRadius,
             nRowsUpdated);

    return true;
}
