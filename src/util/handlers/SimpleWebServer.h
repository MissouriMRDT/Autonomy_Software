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
#include <atomic>
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

        /******************************************************************************
         * @brief Represents a local path as an object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-03-25
         ******************************************************************************/
        struct StaticDir
        {
            public:
                std::string szLocalPath;
        };

        std::map<std::string, StaticDir> m_mStaticDirectories;

        // Thread Management.
        std::thread m_thAcceptThread;
        std::vector<std::thread> m_vWorkerThreads;
        std::mutex m_muThreadMutex;

        ////////////////////////////////////
        // Private Methods
        ////////////////////////////////////

        void StartServer();
        void StopServer();
        void AcceptLoop();
        void HandleClient(int nClientFD);

        // File Utilities.
        std::vector<char> LoadFile(const std::string& szPath);
        std::string GetMimeType(const std::string& szPath);
};

#endif
