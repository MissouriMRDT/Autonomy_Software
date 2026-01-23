/******************************************************************************
 * @brief Implements the SimpleWebServer class utility.
 *
 * @file SimpleWebServer.cpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2026-01-20
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#include "./SimpleWebServer.h"
#include "../../AutonomyLogging.h"

/// \cond
#include <algorithm>
#include <cstring>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

/// \endcond

/******************************************************************************
 * @brief Construct a new Simple Web Server object.
 *
 * @param nPort - Port to listen on.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
SimpleWebServer::SimpleWebServer(int nPort)
{
    // Initialize member variables.
    m_nPort     = nPort;
    m_nSocketFD = -1;
    m_bRunning  = false;

    // Start the server.
    this->StartServer();
}

/******************************************************************************
 * @brief Destroy the Simple Web Server object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
SimpleWebServer::~SimpleWebServer()
{
    // Stop the server.
    this->StopServer();
}

/******************************************************************************
 * @brief Mutator for the Html Content private member.
 *
 * @param sHtml - The HTML content to be sent to clients.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void SimpleWebServer::SetHtmlContent(const std::string& szHtml)
{
    std::lock_guard<std::mutex> lkLock(m_muDataMutex);
    m_szHtmlContent = szHtml;
}

/******************************************************************************
 * @brief Registers a GET endpoint with a callback function.
 *
 * @param szEndpoint - The endpoint to register.
 * @param callback - The callback function to execute when the endpoint is requested.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void SimpleWebServer::RegisterEndpoint(const std::string& szEndpoint, RequestCallback fnCallback)
{
    std::lock_guard<std::mutex> lkLock(m_muDataMutex);
    m_mGetCallbacks[szEndpoint] = fnCallback;
}

/******************************************************************************
 * @brief Starts the web server.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void SimpleWebServer::StartServer()
{
    // Check if already running.
    if (m_bRunning)
    {
        return;
    }

    // Create socket and check for errors.
    int nFD = socket(AF_INET, SOCK_STREAM, 0);
    if (nFD < 0)
    {
        LOG_ERROR(logging::g_qSharedLogger, "WebServer: Failed to create socket.");
        return;
    }
    m_nSocketFD = nFD;

    // Set socket options and bind.
    int nEnable = 1;
    setsockopt(m_nSocketFD.load(), SOL_SOCKET, SO_REUSEADDR, &nEnable, sizeof(int));

    // Setup address structure.
    sockaddr_in stAddr;
    stAddr.sin_family      = AF_INET;
    stAddr.sin_addr.s_addr = INADDR_ANY;
    stAddr.sin_port        = htons(m_nPort);
    // Bind and listen.
    if (::bind(m_nSocketFD.load(), (struct sockaddr*) &stAddr, sizeof(stAddr)) < 0)
    {
        // If bind fails, clean up and log error.
        close(m_nSocketFD.load());
        m_nSocketFD = -1;
        LOG_ERROR(logging::g_qSharedLogger, "WebServer: Failed to bind port {}", m_nPort);
        return;
    }

    // Start listening for connections.
    if (listen(m_nSocketFD.load(), 10) < 0)
    {
        // If listen fails, clean up and log error.
        close(m_nSocketFD.load());
        m_nSocketFD = -1;
        LOG_ERROR(logging::g_qSharedLogger, "WebServer: Failed to listen on port {}", m_nPort);
        return;
    }

    // Set running flag and start accept thread.
    m_bRunning       = true;
    m_thAcceptThread = std::thread(&SimpleWebServer::AcceptLoop, this);
    // Submit logger message.
    LOG_INFO(logging::g_qSharedLogger, "WebServer: Started on port {}", m_nPort);
}

/******************************************************************************
 * @brief Stops the web server.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void SimpleWebServer::StopServer()
{
    // Set running flag to false.
    m_bRunning = false;

    // Shutdown triggers the accept loop to unblock and exit.
    if (m_nSocketFD.load() != -1)
    {
        shutdown(m_nSocketFD.load(), SHUT_RDWR);
        close(m_nSocketFD.load());
        m_nSocketFD = -1;
    }

    // Join accept thread.
    if (m_thAcceptThread.joinable())
    {
        m_thAcceptThread.join();
    }

    // Join all workers to prevent use-after-free.
    std::lock_guard<std::mutex> lk(m_muThreadMutex);
    for (std::thread& thThread : m_vWorkerThreads)
    {
        if (thThread.joinable())
            thThread.join();
    }

    // Clear worker threads vector.
    m_vWorkerThreads.clear();
}

/******************************************************************************
 * @brief Main loop to accept incoming connections.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void SimpleWebServer::AcceptLoop()
{
    // Continue accepting while running.
    while (m_bRunning)
    {
        // Clean up finished threads.
        {
            std::lock_guard<std::mutex> lkThreadLock(m_muThreadMutex);
            std::vector<std::thread>::iterator itIndex = std::remove_if(m_vWorkerThreads.begin(), m_vWorkerThreads.end(), [](std::thread& t) { return !t.joinable(); });
            m_vWorkerThreads.erase(itIndex, m_vWorkerThreads.end());
        }

        // Configure client address structure.
        struct sockaddr_in stClientAddr;
        socklen_t clientLen = sizeof(stClientAddr);
        // Blocking accept call.
        int nClientFD = accept(m_nSocketFD.load(), (struct sockaddr*) &stClientAddr, &clientLen);
        // Check for errors.
        if (nClientFD < 0)
        {
            continue;
        }

        // If client accepted, spawn a new thread to handle it.
        std::lock_guard<std::mutex> lkThreadLock(m_muThreadMutex);
        m_vWorkerThreads.emplace_back(&SimpleWebServer::HandleClient, this, nClientFD);
    }
}

/******************************************************************************
 * @brief Handles an individual client connection.
 *
 * @param nClientFD - File descriptor for the client socket.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-01-22
 ******************************************************************************/
void SimpleWebServer::HandleClient(int nClientFD)
{
    // Timeout to prevent stuck threads.
    struct timeval stTimeVal;
    stTimeVal.tv_sec  = 5;
    stTimeVal.tv_usec = 0;
    setsockopt(nClientFD, SOL_SOCKET, SO_RCVTIMEO, (const char*) &stTimeVal, sizeof stTimeVal);

    // Create buffer and read request.
    std::vector<char> vRawRequest;
    char aChunk[1024];
    bool bHeaderFound = false;

    // Read until we find the double CRLF.
    while (!bHeaderFound && m_bRunning)
    {
        // Read chunk.
        ssize_t siBytes = recv(nClientFD, aChunk, sizeof(aChunk), 0);
        // Check for errors or disconnection.
        if (siBytes <= 0)
        {
            break;
        }
        // Append to raw request buffer.
        vRawRequest.insert(vRawRequest.end(), aChunk, aChunk + siBytes);

        // Check for end of headers.
        std::string szCurrent(vRawRequest.begin(), vRawRequest.end());
        // If we find the header terminator, set flag.
        if (szCurrent.find("\r\n\r\n") != std::string::npos)
        {
            bHeaderFound = true;
        }

        // Safety limit to prevent overly large requests.
        if (vRawRequest.size() > 16384)
        {
            break;    // Safety limit.
        }
    }

    // If we found a complete header, process the request.
    if (bHeaderFound && m_bRunning)
    {
        // Parse request line.
        std::string szRequest(vRawRequest.begin(), vRawRequest.end());
        std::string szMethod, szPath, szQuery;
        std::istringstream stdISS(szRequest);
        stdISS >> szMethod >> szPath;

        // Separate query string if present.
        size_t siQPos = szPath.find('?');
        // If found, split path and query.
        if (siQPos != std::string::npos)
        {
            szQuery = szPath.substr(siQPos + 1);
            szPath  = szPath.substr(0, siQPos);
        }

        // Create callback and HTML copy variables to use outside lock.
        RequestCallback fnCallback = nullptr;
        std::string szHtmlCopy;

        {
            // Lock data mutex to access callbacks and HTML content.
            std::lock_guard<std::mutex> lkDataLock(m_muDataMutex);
            // Check if a callback is registered for the requested path.
            if (m_mGetCallbacks.count(szPath))
            {
                // If so, retrieve it.
                fnCallback = m_mGetCallbacks[szPath];
            }
            else
            {
                // Otherwise, copy the HTML content.
                szHtmlCopy = m_szHtmlContent;
            }
        }

        // Check if we have a callback or HTML to serve.
        if (fnCallback)
        {
            // Create response body from callback.
            std::vector<char> vData = fnCallback(szQuery);
            std::string szHeader    = "HTTP/1.1 200 OK\r\n"
                                      "Content-Type: application/octet-stream\r\n"
                                      "Access-Control-Allow-Origin: *\r\n"
                                      "Content-Length: " +
                                   std::to_string(vData.size()) +
                                   "\r\n"
                                   "Connection: close\r\n\r\n";
            // Send header.
            send(nClientFD, szHeader.c_str(), szHeader.size(), MSG_NOSIGNAL);

            // Send body in chunks
            size_t siRemaining = vData.size();
            size_t siSent      = 0;
            while (siRemaining > 0 && m_bRunning)
            {
                // Calculate chunk size and send.
                size_t siChunk = (siRemaining > 65536) ? 65536 : siRemaining;
                ssize_t result = send(nClientFD, vData.data() + siSent, siChunk, MSG_NOSIGNAL);
                // Check for errors.
                if (result <= 0)
                {
                    break;
                }

                // Update counters.
                siSent += result;
                siRemaining -= result;
            }
        }
        // If no callback, serve HTML if requested.
        else if (szPath == "/" || szPath == "/index.html")
        {
            // Create and send HTTP response with HTML content.
            std::string szHeader = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: " + std::to_string(szHtmlCopy.size()) + "\r\nConnection: close\r\n\r\n";
            send(nClientFD, szHeader.c_str(), szHeader.size(), MSG_NOSIGNAL);
            send(nClientFD, szHtmlCopy.c_str(), szHtmlCopy.size(), MSG_NOSIGNAL);
        }
        // If neither, respond with 404.
        else
        {
            // Send 404 Not Found response.
            std::string szResp = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
            send(nClientFD, szResp.c_str(), szResp.size(), MSG_NOSIGNAL);
        }
    }

    // Close client connection.
    close(nClientFD);
}
