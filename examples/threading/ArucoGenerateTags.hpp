/******************************************************************************
 * @brief This example uses the OpenCV TagGenerator.hpp example and multithreads
 *      the generation of all available Aruco tags for the given dictionary type.
 *
 * @file ArucoGenerateTags.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-23
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include <chrono>
#include <filesystem>
#include <iostream>

#include "../../src/interfaces/AutonomyThread.hpp"
#include "../../src/util/ExampleChecker.h"
#include "../opencv/TagGenerator.hpp"

/******************************************************************************
 * @brief This class inherits the AutonomyThread interface and implements the threaded
 * container methods. It also utilizes the ability to create a thread pool
 * of subroutines and the ability to parallelize loops.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-27
 ******************************************************************************/
class ArucoGenerateTagsThreaded : public AutonomyThread<void>
{
    private:
        // Declare and define private member variables.
        int m_nNumTagsToGenerate = 0;
        std::vector<cv::aruco::PredefinedDictionaryType> m_vDictionaries;
        std::mutex m_muDictMutex;    // This mutex is used for locking write access to the dictionary in threads.

        /******************************************************************************
         * @brief This code will run in a separate thread. This is the main code.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-23
         ******************************************************************************/
        void ThreadedContinuousCode() override
        {
            // Start thread pool. Run detached since the threads aren't returning anything.
            // This is much faster than the normal RunPool function.
            this->RunDetachedPool(m_vDictionaries.size(), m_vDictionaries.size());

            // Wait for pool tasks to finish.
            this->JoinPool();

            // Stop threaded code.
            this->RequestStop();
        }

        /******************************************************************************
         * @brief Any highly parallelizable code that can be used in the main thread
         *       goes here.
         *
         *      This is not required and is only used at the users convenience.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-25
         ******************************************************************************/
        void PooledLinearCode() override
        {
            // Acquire resource lock for dictionary vector.
            std::unique_lock<std::mutex> lock(m_muDictMutex);
            // Get dictionary enum from back of dictionary vector.
            cv::aruco::PredefinedDictionaryType cvDictType = m_vDictionaries.back();
            m_vDictionaries.pop_back();
            // Release resource lock.
            lock.unlock();

            //////////////////////////////////////////////////
            // Change this bool to test the two different
            // blocks of code!
            //////////////////////////////////////////////////
            bool bUseParallelLoop = false;
            if (bUseParallelLoop)
            {
                //////////////////////////////////////////////////
                // This code splits the loop into parts and
                // completes it in different threads.
                //////////////////////////////////////////////////
                this->ParallelizeLoop(50,
                                      m_nNumTagsToGenerate,
                                      [&cvDictType](const int a, const int b)
                                      {
                                          // Loop through and generate each of the tags.
                                          for (int i = a; i < b; ++i)
                                              GenerateOpenCVArucoMarker(cvDictType, i);
                                      });
            }
            else
            {
                //////////////////////////////////////////////////
                // This code linearly runs through every tag.
                //////////////////////////////////////////////////
                for (int i = 0; i < m_nNumTagsToGenerate; ++i)
                {
                    GenerateOpenCVArucoMarker(cvDictType, i);
                }
            }
        }

    public:
        // Declare and define public methods and variables.
        ArucoGenerateTagsThreaded() = default;

        /******************************************************************************
         * @brief Mutator for the Num Tags To Generate private member. Given number
         *      must not exceed the dictionary type limit.
         *
         * @param nNumTags - The number of tags to generate.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-23
         ******************************************************************************/
        void SetNumTagsToGenerate(const int nNumTags) { m_nNumTagsToGenerate = nNumTags; }

        /******************************************************************************
         * @brief Mutator for the Tag Dictionary Type private member
         *
         * @param eDictionary - The dictionary type to use for generation.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-23
         ******************************************************************************/
        void AddTagDictionaryType(const cv::aruco::PredefinedDictionaryType eDictionary) { m_vDictionaries.emplace_back(eDictionary); }
};

/******************************************************************************
 * @brief This class is non threaded.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-27
 ******************************************************************************/
class ArucoGenerateTagsLinear
{
    private:
        // Declare and define private member variables.
        int m_nNumTagsToGenerate = 0;
        std::vector<cv::aruco::PredefinedDictionaryType> m_vDictionaries;

    public:
        // Declare and define public methods and variables.
        ArucoGenerateTagsLinear() = default;

        /******************************************************************************
         * @brief Single threaded, runs in main loop.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-25
         ******************************************************************************/
        void Start()
        {
            while (m_vDictionaries.size() > 0)
            {
                // Get dictionary enum from back of dictionary vector.
                cv::aruco::PredefinedDictionaryType cvDictType = m_vDictionaries.back();
                m_vDictionaries.pop_back();

                // Loop through and generate each of the tags.
                for (int i = 0; i < m_nNumTagsToGenerate; ++i)
                    GenerateOpenCVArucoMarker(cvDictType, i);
            }
        }

        /******************************************************************************
         * @brief Mutator for the Num Tags To Generate private member. Given number
         *      must not exceed the dictionary type limit.
         *
         * @param nNumTags - The number of tags to generate.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-23
         ******************************************************************************/
        void SetNumTagsToGenerate(const int nNumTags) { m_nNumTagsToGenerate = nNumTags; }

        /******************************************************************************
         * @brief Mutator for the Tag Dictionary Type private member
         *
         * @param eDictionary - The dictionary type to use for generation.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-07-23
         ******************************************************************************/
        void AddTagDictionaryType(const cv::aruco::PredefinedDictionaryType eDictionary) { m_vDictionaries.emplace_back(eDictionary); }
};

/******************************************************************************
 * @brief Main example method.
 *
 * @return int - Return status.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-23
 ******************************************************************************/
void RunExample()
{
    ////////////////////////////////////
    ///  Linear generator.
    ////////////////////////////////////
    // Start the timer
    auto tmStartTime = std::chrono::high_resolution_clock::now();

    // Create generator.
    ArucoGenerateTagsLinear LinearTagGenerator;

    // Configure tag generator for first run.
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_50);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_50);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_50);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_50);
    LinearTagGenerator.SetNumTagsToGenerate(50);
    // Start first run.
    LinearTagGenerator.Start();

    // Add new dictionary to generator.
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_100);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_100);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_100);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_100);
    LinearTagGenerator.SetNumTagsToGenerate(100);
    // Start second run.
    LinearTagGenerator.Start();

    // Add new dictionary to generator.
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_250);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_250);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_250);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_250);
    LinearTagGenerator.SetNumTagsToGenerate(250);
    // Start third run.
    LinearTagGenerator.Start();

    // Add new dictionary to generator.
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_1000);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_1000);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_1000);
    LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_1000);
    LinearTagGenerator.SetNumTagsToGenerate(1000);
    // Start fourth run.
    LinearTagGenerator.Start();

    // End the timer
    auto tmEndTime = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    auto singleThreadedDuration = std::chrono::duration_cast<std::chrono::milliseconds>(tmEndTime - tmStartTime).count();
    // Print the result
    std::cout << "Time taken for single threaded generator: " << singleThreadedDuration << " milliseconds." << std::endl;

    // Remove all png files in the current directory.
    for (const auto& entry : std::filesystem::directory_iterator("."))
    {
        // Loop all *.png files.
        if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ".png")
        {
            // Delete.
            std::filesystem::remove(entry.path());
        }
    }

    ////////////////////////////////////
    ///  Threaded generator.
    ////////////////////////////////////
    // Start the timer
    tmStartTime = std::chrono::high_resolution_clock::now();

    // Create Aruco generator threads.
    ArucoGenerateTagsThreaded ThreadedTagGenerator1;
    ArucoGenerateTagsThreaded ThreadedTagGenerator2;
    ArucoGenerateTagsThreaded ThreadedTagGenerator3;
    ArucoGenerateTagsThreaded ThreadedTagGenerator4;

    // Configure tag generator for first run.
    ThreadedTagGenerator1.AddTagDictionaryType(cv::aruco::DICT_4X4_50);
    ThreadedTagGenerator1.AddTagDictionaryType(cv::aruco::DICT_5X5_50);
    ThreadedTagGenerator1.AddTagDictionaryType(cv::aruco::DICT_6X6_50);
    ThreadedTagGenerator1.AddTagDictionaryType(cv::aruco::DICT_7X7_50);
    ThreadedTagGenerator1.SetNumTagsToGenerate(50);
    // Start first run.
    ThreadedTagGenerator1.Start();

    // Add new dictionary to generator.
    ThreadedTagGenerator2.AddTagDictionaryType(cv::aruco::DICT_4X4_100);
    ThreadedTagGenerator2.AddTagDictionaryType(cv::aruco::DICT_5X5_100);
    ThreadedTagGenerator2.AddTagDictionaryType(cv::aruco::DICT_6X6_100);
    ThreadedTagGenerator2.AddTagDictionaryType(cv::aruco::DICT_7X7_100);
    ThreadedTagGenerator2.SetNumTagsToGenerate(100);
    // Start second run.
    ThreadedTagGenerator2.Start();

    // Add new dictionary to generator.
    ThreadedTagGenerator3.AddTagDictionaryType(cv::aruco::DICT_4X4_250);
    ThreadedTagGenerator3.AddTagDictionaryType(cv::aruco::DICT_5X5_250);
    ThreadedTagGenerator3.AddTagDictionaryType(cv::aruco::DICT_6X6_250);
    ThreadedTagGenerator3.AddTagDictionaryType(cv::aruco::DICT_7X7_250);
    ThreadedTagGenerator3.SetNumTagsToGenerate(250);
    // Start third run.
    ThreadedTagGenerator3.Start();

    // Add new dictionary to generator.
    ThreadedTagGenerator4.AddTagDictionaryType(cv::aruco::DICT_4X4_1000);
    ThreadedTagGenerator4.AddTagDictionaryType(cv::aruco::DICT_5X5_1000);
    ThreadedTagGenerator4.AddTagDictionaryType(cv::aruco::DICT_6X6_1000);
    ThreadedTagGenerator4.AddTagDictionaryType(cv::aruco::DICT_7X7_1000);
    ThreadedTagGenerator4.SetNumTagsToGenerate(1000);
    // Start fourth run.
    ThreadedTagGenerator4.Start();

    // Join all.
    ThreadedTagGenerator1.Join();
    ThreadedTagGenerator2.Join();
    ThreadedTagGenerator3.Join();
    ThreadedTagGenerator4.Join();

    // End the timer
    tmEndTime = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    auto tmMultiThreadedDuration = std::chrono::duration_cast<std::chrono::milliseconds>(tmEndTime - tmStartTime).count();
    // Print the result
    std::cout << "Time taken for multithreaded generator: " << tmMultiThreadedDuration << " milliseconds." << std::endl;
    std::cout << "\n\nCheck your build directory for all aruco tags!" << std::endl;
}
