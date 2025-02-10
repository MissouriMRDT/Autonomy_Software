/******************************************************************************
 * @brief Base class for all tests. Includes default setup and teardown methods.
 *
 * @file TestingBase.hh
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef TESTING_BASE_HH
#define TESTING_BASE_HH

#include "../src/AutonomyGlobals.h"
#include "../src/AutonomyLogging.h"
#include "../src/AutonomyNetworking.h"

/// \cond
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Base class for all tests.
 *
 * @tparam T - Type of the test class.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-09
 ******************************************************************************/
template<typename T>
class TestingBase : public ::testing::Test
{
    public:
        /******************************************************************************
         * @brief Construct a new Testing Base object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        TestingBase() = default;

        /******************************************************************************
         * @brief Destroy the Testing Base object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        virtual ~TestingBase() = default;

        /******************************************************************************
         * @brief Mutator for the Command Line Args private member
         *
         * @param argc - The number of command line arguments
         * @param argv - The command line arguments
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        inline static void SetCommandLineArgs(int argc, char** argv)
        {
            m_argc = argc;
            m_argv = argv;
        }

    protected:
        /******************************************************************************
         * @brief Required setup for all tests.
         *
         *        This method initializes the loggers and RoveComm instances.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        inline void RequiredSetup()
        {
            // Parse the command line arguments.
            ParseArgc();

            // Initialize Loggers if they are not already initialized.
            if (logging::g_qSharedLogger == nullptr)
            {
                logging::InitializeLoggers(constants::LOGGING_OUTPUT_PATH_ABSOLUTE, m_szTimestamp);
            }

            // Initialize RoveComm.
            if (network::g_pRoveCommUDPNode == nullptr)
            {
                network::g_pRoveCommUDPNode   = new rovecomm::RoveCommUDP();
                network::g_bRoveCommUDPStatus = network::g_pRoveCommUDPNode->InitUDPSocket(manifest::General::ETHERNET_UDP_PORT);
            }

            if (network::g_pRoveCommTCPNode == nullptr)
            {
                network::g_pRoveCommTCPNode = new rovecomm::RoveCommTCP();
                network::g_bRoveCommTCPStatus =
                    network::g_pRoveCommTCPNode->InitTCPSocket(constants::ROVECOMM_TCP_INTERFACE_IP.c_str(), manifest::General::ETHERNET_TCP_PORT);
            }

            // Check if RoveComm was successfully initialized.
            if (!network::g_bRoveCommUDPStatus || !network::g_bRoveCommTCPStatus)
            {
                // Submit logger message.
                LOG_CRITICAL(logging::g_qSharedLogger,
                             "RoveComm did not initialize properly! UDPNode Status: {}, TCPNode Status: {}",
                             network::g_bRoveCommUDPStatus,
                             network::g_bRoveCommTCPStatus);

                // Since RoveComm is crucial, skip the test.
                GTEST_SKIP();
            }
            else
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "RoveComm UDP and TCP nodes successfully initialized.");
            }
        }

        /******************************************************************************
         * @brief Required teardown for all tests.
         *
         *        This method stops the RoveComm instances and loggers.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        inline void RequiredTeardown()
        {
            // Stop RoveComm quill logging or quill will segfault if trying to output logs to RoveComm.
            network::g_bRoveCommUDPStatus = false;
            network::g_bRoveCommTCPStatus = false;

            // Stop handlers.
            if (network::g_pRoveCommUDPNode != nullptr)
            {
                network::g_pRoveCommUDPNode->CloseUDPSocket();
            }
            if (network::g_pRoveCommTCPNode != nullptr)
            {
                network::g_pRoveCommTCPNode->CloseTCPSocket();
            }

            // Delete dynamically allocated objects.
            delete network::g_pRoveCommUDPNode;
            delete network::g_pRoveCommTCPNode;

            // Set dangling pointers to null.
            network::g_pRoveCommUDPNode = nullptr;
            network::g_pRoveCommTCPNode = nullptr;
        }

        /******************************************************************************
         * @brief User's setup method.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-06
         ******************************************************************************/
        virtual inline void TestSetup() {}

        /******************************************************************************
         * @brief User's teardown method.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-06
         ******************************************************************************/
        virtual inline void TestTeardown() {}

    private:
        // Declare private members.
        static int m_argc;
        static char** m_argv;
        static std::string m_szTimestamp;

        /******************************************************************************
         * @brief Method to set up the test class.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        inline void SetUp() override
        {
            // Call the base setup method. This initializes the loggers and RoveComm instances.
            RequiredSetup();
            // Call the user's setup method.
            TestSetup();
        }

        /******************************************************************************
         * @brief Method to tear down the test class.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        inline void TearDown() override
        {
            // Call the base teardown method. This stops the RoveComm instances and loggers.
            RequiredTeardown();
            // Call the user's teardown method.
            TestTeardown();
        }

        /******************************************************************************
         * @brief Method to parse the command line arguments.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-09
         ******************************************************************************/
        inline void ParseArgc()
        {
            // Set the current timestamp as the default timestamp.
            std::string szTimestamp = timeops::GetTimestamp();

            // Check if the test being run is in the list of tests to skip setting up logging.
            for (int i = 1; i < m_argc; ++i)
            {
                if (std::string(m_argv[i]).find("--timestamp=") != std::string::npos)
                {
                    szTimestamp = std::string(m_argv[i]).substr(std::string(m_argv[i]).find("=") + 1);
                    break;
                }
            }

            // Set the timestamp.
            m_szTimestamp = szTimestamp;
        }
};

// Initialize static members
template<typename T>
int TestingBase<T>::m_argc = 0;

template<typename T>
char** TestingBase<T>::m_argv = nullptr;

template<typename T>
std::string TestingBase<T>::m_szTimestamp = "";

#endif    // TESTING_BASE_HH
