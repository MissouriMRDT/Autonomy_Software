#include <iostream>
#include <sqlite3.h>
#include <string>

const char* DB_PATH = "../../data/LiDAR/sqlite/MDRS.db";

/******************************************************************************
 * @brief Creates a SQLite database at the specified path.
 *
 *        Opens or creates the database file located at the path defined by DB_PATH.
 *        Closes the database after verifying access.
 *
 * @return int - Returns 0 on success, -1 on failure.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-04-19
 ******************************************************************************/
int CreateDB()
{
    sqlite3* DB;
    int exit = sqlite3_open(DB_PATH, &DB);

    if (exit != SQLITE_OK)
    {
        std::cerr << "Error opening DB: " << sqlite3_errmsg(DB) << std::endl;
        return -1;
    }

    std::cout << "Opened Database Successfully!" << std::endl;
    sqlite3_close(DB);
    return 0;
}

/******************************************************************************
 * @brief Creates the required tables for LiDAR point data storage.
 *
 *        This function creates the following tables if they do not already exist:
 *        - RawPoints: Stores UTM point data with classification and nearby points.
 *        - RoughnessClassifications: Links classification labels to roughness values.
 *        - GroundPoints: Stores only ground points with roughness metadata.
 *        - NonGroundPoints: Stores all other points not classified as ground.
 *
 * @return int - Returns 0 on success, Other on failure.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-04-19
 ******************************************************************************/
int CreateTables()
{
    sqlite3* DB;
    int exit = sqlite3_open(DB_PATH, &DB);

    if (exit != SQLITE_OK)
    {
        std::cerr << "Can't open database: " << sqlite3_errmsg(DB) << std::endl;
        return exit;
    }

    const std::string rawPointsTable       = R"(
        CREATE TABLE IF NOT EXISTS RawPoints (
            Id TEXT PRIMARY KEY,
            Easting REAL NOT NULL,
            Northing REAL NOT NULL,
            Altitude REAL NOT NULL,
            Zone TEXT NOT NULL,
            Classification TEXT NOT NULL,
            NearbyPoints TEXT
        );
    )";

    const std::string roughnessClassTable  = R"(
        CREATE TABLE IF NOT EXISTS RoughnessClassifications (
            Classification TEXT PRIMARY KEY,
            Roughness REAL
        );
    )";

    const std::string groundPointsTable    = R"(
        CREATE TABLE IF NOT EXISTS GroundPoints (
            PointId TEXT PRIMARY KEY,
            Roughness REAL,
            FOREIGN KEY (PointId) REFERENCES RawPoints(Id)
        );
    )";

    const std::string nonGroundPointsTable = R"(
        CREATE TABLE IF NOT EXISTS NonGroundPoints (
            PointId TEXT PRIMARY KEY,
            FOREIGN KEY (PointId) REFERENCES RawPoints(Id)
        );
    )";

    char* errorMessage                     = nullptr;

    auto exec_sql                          = [&](const std::string& sql, const std::string& tableName)
    {
        exit = sqlite3_exec(DB, sql.c_str(), nullptr, nullptr, &errorMessage);
        if (exit != SQLITE_OK)
        {
            std::cerr << "Error creating table [" << tableName << "]: " << errorMessage << std::endl;
            sqlite3_free(errorMessage);
        }
        else
        {
            std::cout << "Table [" << tableName << "] created successfully." << std::endl;
        }
    };

    exec_sql(rawPointsTable, "RawPoints");
    exec_sql(roughnessClassTable, "RoughnessClassifications");
    exec_sql(groundPointsTable, "GroundPoints");
    exec_sql(nonGroundPointsTable, "NonGroundPoints");

    sqlite3_close(DB);
    return 0;
}

/******************************************************************************
 * @brief Callback function for SQLite exec to print the results of a query.
 *
 *        This function is passed to SQLite to print each row of a query result.
 *        It prints column names and their values to stdout.
 *
 * @param data - Label or descriptor for the result set.
 * @param argc - Number of columns.
 * @param argv - Array of column values as C strings.
 * @param azColName - Array of column names.
 * @return int - Returns 0 (always succeeds).
 *
 * @see sqlite3_exec
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-04-19
 ******************************************************************************/
static int Callback(void* data, int argc, char** argv, char** azColName)
{
    std::cout << static_cast<const char*>(data) << ":\n";
    for (int i = 0; i < argc; i++)
    {
        std::cout << "  " << azColName[i] << " = " << (argv[i] ? argv[i] : "NULL") << "\n";
    }
    std::cout << std::endl;
    return 0;
}

/******************************************************************************
 * @brief Inserts hardcoded example raw points into the RawPoints table.
 *
 *        Adds three points with varying classifications and neighbor references.
 *        Each entry is uniquely identified by its ID (Easting_Northing).
 *
 * @return int - Returns 0 on success, non-zero on failure.
 *
 * @note This is a demonstration insert. Replace with real data import logic.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-04-19
 ******************************************************************************/
int InsertRawPoints()
{
    sqlite3* DB;
    int exit = sqlite3_open(DB_PATH, &DB);
    if (exit != SQLITE_OK)
    {
        std::cerr << "Can't open database for insert.\n";
        return exit;
    }

    std::string sql = R"(
        INSERT INTO RawPoints VALUES ('500000_4100000', 500000, 4100000, 150.0, '15T', 'ground', '500001_4100001,500002_4100002');
        INSERT INTO RawPoints VALUES ('500001_4100001', 500001, 4100001, 151.2, '15T', 'ground', '500000_4100000');
        INSERT INTO RawPoints VALUES ('500002_4100002', 500002, 4100002, 148.9, '15T', 'rock', '500000_4100000');
    )";

    char* errorMessage;
    exit = sqlite3_exec(DB, sql.c_str(), nullptr, nullptr, &errorMessage);
    if (exit != SQLITE_OK)
    {
        std::cerr << "Error inserting raw points: " << errorMessage << std::endl;
        sqlite3_free(errorMessage);
    }
    else
    {
        std::cout << "RawPoints inserted successfully.\n";
    }

    sqlite3_close(DB);
    return exit;
}

/******************************************************************************
 * @brief Deletes a raw point from the RawPoints table by ID.
 *
 *        Performs a SQL DELETE operation based on the provided point ID.
 *
 * @param id - The ID of the raw point to delete (formatted as Easting_Northing).
 * @return int - Returns 0 on success, non-zero on failure.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-04-19
 ******************************************************************************/
int DeleteRawPointById(const std::string& id)
{
    sqlite3* DB;
    int exit = sqlite3_open(DB_PATH, &DB);
    if (exit != SQLITE_OK)
    {
        std::cerr << "Can't open database for delete.\n";
        return exit;
    }

    std::string sql = "DELETE FROM RawPoints WHERE Id = '" + id + "';";
    char* errorMessage;
    exit = sqlite3_exec(DB, sql.c_str(), nullptr, nullptr, &errorMessage);
    if (exit != SQLITE_OK)
    {
        std::cerr << "Error deleting point: " << errorMessage << std::endl;
        sqlite3_free(errorMessage);
    }
    else
    {
        std::cout << "Point '" << id << "' deleted successfully.\n";
    }

    sqlite3_close(DB);
    return exit;
}

/******************************************************************************
 * @brief Prints the contents of the RawPoints table.
 *
 *        Executes a SELECT * query on RawPoints and prints the result to stdout.
 *
 * @return int - Returns 0 on success, non-zero on failure.
 *
 * @see Callback
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-04-19
 ******************************************************************************/
int PrintRawPoints()
{
    sqlite3* DB;
    int exit = sqlite3_open(DB_PATH, &DB);
    if (exit != SQLITE_OK)
    {
        std::cerr << "Can't open database for reading.\n";
        return exit;
    }

    std::string sql = "SELECT * FROM RawPoints;";
    char* errorMessage;
    exit = sqlite3_exec(DB, sql.c_str(), Callback, (void*) "RawPoints Table", &errorMessage);
    if (exit != SQLITE_OK)
    {
        std::cerr << "Error reading RawPoints: " << errorMessage << std::endl;
        sqlite3_free(errorMessage);
    }

    sqlite3_close(DB);
    return exit;
}

/******************************************************************************
 * @brief Selects all records from a specified table and prints them.
 *
 *        Executes a generic SELECT * FROM [tableName] and uses a callback
 *        to display the results.
 *
 * @param tableName - Name of the SQLite table to query.
 * @return int - Returns 0 on success, non-zero on failure.
 *
 * @note Table name must exist in the schema or an error will be shown.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-04-19
 ******************************************************************************/
int SelectFromTable(const std::string& tableName)
{
    sqlite3* DB;
    int exit = sqlite3_open(DB_PATH, &DB);
    if (exit != SQLITE_OK)
    {
        std::cerr << "Error opening database for SELECT.\n";
        return exit;
    }

    std::string sql    = "SELECT * FROM " + tableName + ";";
    std::string label  = "RESULTS FROM " + tableName;
    char* errorMessage = nullptr;

    exit               = sqlite3_exec(DB, sql.c_str(), Callback, (void*) label.c_str(), &errorMessage);
    if (exit != SQLITE_OK)
    {
        std::cerr << "Error selecting from " << tableName << ": " << errorMessage << std::endl;
        sqlite3_free(errorMessage);
    }

    sqlite3_close(DB);
    return exit;
}

/******************************************************************************
 * @brief Main function to create the database, tables, and perform operations.
 *
 *        Initializes the database and tables, demonstrates insertion and deletion
 *        of raw LiDAR points, and prints table contents to validate changes.
 *
 * @return int - Returns 0 on success, Other on failure.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-04-19
 ******************************************************************************/
int main()
{
    std::cout << "Creating database..." << std::endl;
    if (CreateDB() != 0)
        return -1;

    std::cout << "Creating tables..." << std::endl;
    if (CreateTables() != 0)
        return -1;

    std::cout << "\nSTATE OF RawPoints BEFORE INSERT" << std::endl;
    PrintRawPoints();

    std::cout << "\nInserting RawPoints..." << std::endl;
    InsertRawPoints();

    std::cout << "\nSTATE OF RawPoints AFTER INSERT" << std::endl;
    PrintRawPoints();

    std::cout << "\nDeleting point '500001_4100001'..." << std::endl;
    DeleteRawPointById("500001_4100001");

    std::cout << "\nSTATE OF RawPoints AFTER DELETE" << std::endl;
    PrintRawPoints();

    std::cout << "\nQuerying GroundPoints Table..." << std::endl;
    SelectFromTable("GroundPoints");

    std::cout << "\nDatabase and operations completed successfully." << std::endl;
    return 0;
}
