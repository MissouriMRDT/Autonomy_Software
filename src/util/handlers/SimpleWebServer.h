/******************************************************************************
 * @brief Defines the SimpleWebServer class.
 *
 * @file SimpleWebServer.h
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2026-01-20
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#ifndef SIMPLEWEBSERVER_H
#define SIMPLEWEBSERVER_H

/// \cond
#include "../../../external/threadpool/include/BS_thread_pool.hpp"
#include <atomic>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief A lightweight, multi-threaded HTTP server.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2026-01-20
 ******************************************************************************/
class SimpleWebServer
{
    public:
        using RequestCallback = std::function<std::vector<char>(const std::string&)>;

        ////////////////////////////////////
        // Declare class methods.
        ////////////////////////////////////

        SimpleWebServer(int nPort = 8080);
        ~SimpleWebServer();
        void SetHtmlContent(const std::string& szHtml);
        void RegisterEndpoint(const std::string& szEndpoint, RequestCallback fnCallback);
        void AddStaticDirectory(const std::string& szUrlPrefix, const std::string& szLocalDir);

    private:
        ////////////////////////////////////
        // Private Members
        ////////////////////////////////////

        int m_nPort;
        std::atomic<int> m_nSocketFD;
        std::atomic<bool> m_bRunning;
        std::string m_szHtmlContent;
        std::map<std::string, RequestCallback> m_mGetCallbacks;
        std::mutex m_muDataMutex;

        std::map<std::string, std::filesystem::path> m_mStaticDirectories;

        // Thread Management.
        std::thread m_thAcceptThread;
        BS::thread_pool<> m_tpWorkerPool = BS::thread_pool<>(1);
        std::mutex m_muThreadMutex;

        ////////////////////////////////////
        // Private Methods
        ////////////////////////////////////

        void StartServer();
        void StopServer();
        void AcceptLoop();
        void HandleClient(int nClientFD);

        // File Utilities.
        std::vector<char> LoadFile(const std::filesystem::path& szPath);
        std::string GetMimeType(const std::filesystem::path& szPath);
};

#endif
