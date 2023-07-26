/******************************************************************************
 * @brief This example uses the opencv TagGenerator.hpp example and multithreads
 *      the generation of all available Aruco tags for the given dictionary type.
 *
 * @file ArucoGenerateTags.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0723
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/
#include "../../src/interfaces/AutonomyThread.hpp"
#include "../opencv/TagGenerator.hpp"

/******************************************************************************
 * This class inherits the AutonomyThread interface and implments the threaded
 * container methods. It also utilizes the ability to create a thread pool
 * of subroutines and the ability to parallelize loops.
 ******************************************************************************/
class ArucoGenerateTagsThreaded : public AutonomyThread<void>
{
    private:
        // Declare and define private member variables.
        int m_nNumTagsToGenerate = 0;
        std::vector<cv::aruco::PredefinedDictionaryType> m_vDictionaries;

        /******************************************************************************
         * @brief This code will run in a seperate thread. This is the main code.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        void ThreadedContinuousCode()
        {
            // Start thread pool. Run detached since the threads aren't returning anything.
            // This is much faster than the normal RunPool function.
            this->RunDetachedPool(m_vDictionaries.size(), true);

            // Wait for pool tasks to finish.
            this->JoinPool();

            // Stop threaded code.
            this->RequestStop();
        }

        /******************************************************************************
         * @brief Any highly parallelizable code that can be used in the main thread
         *       goes here.
         *
         *      This is not required and is only used at the users convienence.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0725
         ******************************************************************************/
        void PooledLinearCode()
        {
            // Get dictionary enum from back of dictionary vector.
            cv::aruco::PredefinedDictionaryType cvDictType = m_vDictionaries.back();
            m_vDictionaries.pop_back();

            // Parallelize loop.
            this->ParallelizeLoop(
                m_nNumTagsToGenerate,
                [&cvDictType](const int a, const int b)
                {
                    // Loop through and generate each of the tags.
                    for (int i = a; i < b; ++i)
                        GenerateOpenCVArucoMarker(cvDictType, i);
                },
                true);

            // // Loop through and generate each of the tags. NONPARALLELIZED
            // for (int i = 0; i < m_nNumTagsToGenerate; ++i)
            // {
            //     GenerateOpenCVArucoMarker(cvDictType, i);
            // }
        }

    public:
        // Declare and define public methods and variables.
        ArucoGenerateTagsThreaded() = default;

        /******************************************************************************
         * @brief Mutator for the Num Tags To Generate private member. Given number
         *      must not excede the dictionary type limit.
         *
         * @param nNumTags - The number of tags to generate.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        void SetNumTagsToGenerate(const int nNumTags) { m_nNumTagsToGenerate = nNumTags; }

        /******************************************************************************
         * @brief Mutator for the Tag Dictionary Type private member
         *
         * @param eDictionary - The dictionary type to use for generation.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        void AddTagDictionaryType(const cv::aruco::PredefinedDictionaryType eDictionary) { m_vDictionaries.emplace_back(eDictionary); }
};

/******************************************************************************
 *  This class is non threaded.
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
         * @date 2023-0725
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
         *      must not excede the dictionary type limit.
         *
         * @param nNumTags - The number of tags to generate.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        void SetNumTagsToGenerate(const int nNumTags) { m_nNumTagsToGenerate = nNumTags; }

        /******************************************************************************
         * @brief Mutator for the Tag Dictionary Type private member
         *
         * @param eDictionary - The dictionary type to use for generation.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        void AddTagDictionaryType(const cv::aruco::PredefinedDictionaryType eDictionary) { m_vDictionaries.emplace_back(eDictionary); }
};

/******************************************************************************
 * @brief Main example method.
 *
 * @return int - Return status.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0723
 ******************************************************************************/
int RunExample()
{
    // ////////////////////////////////////
    // ///  Linear generator.
    // ////////////////////////////////////
    // ArucoGenerateTagsLinear LinearTagGenerator;

    // // Configure tag generator.
    // LinearTagGenerator.SetNumTagsToGenerate(50);
    // LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_50);
    // LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_50);
    // LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_50);
    // LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_50);

    // // Start first run.
    // LinearTagGenerator.Start();

    // // Add new dictionary to generator.
    // LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_1000);
    // LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_1000);
    // LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_1000);
    // LinearTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_1000);
    // // Set new num tags to generate.
    // LinearTagGenerator.SetNumTagsToGenerate(1000);

    // // Start second run.
    // LinearTagGenerator.Start();

    ////////////////////////////////////
    ///  Threaded generator.
    ////////////////////////////////////
    // Create Aruco generator threads.
    ArucoGenerateTagsThreaded ThreadedTagGenerator;

    // Configure tag generator.
    ThreadedTagGenerator.SetNumTagsToGenerate(49);
    ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_50);
    ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_50);
    ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_50);
    ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_50);

    // // Start first run.
    ThreadedTagGenerator.Start();
    ThreadedTagGenerator.Join();

    // Add new dictionary to generator.
    ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_1000);
    // ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_250);
    // ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_4X4_100);
    ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_1000);
    // ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_250);
    // ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_5X5_100);
    ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_1000);
    // ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_250);
    // ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_6X6_100);
    ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_1000);
    // ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_250);
    // ThreadedTagGenerator.AddTagDictionaryType(cv::aruco::DICT_7X7_100);
    // Set new num tags to generate.
    ThreadedTagGenerator.SetNumTagsToGenerate(999);

    // Start second run.
    ThreadedTagGenerator.Start();
    ThreadedTagGenerator.Join();

    return 0;
}
