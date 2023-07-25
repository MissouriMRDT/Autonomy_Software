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

class ArucoGenerateTags : public AutonomyThread<void>
{
    private:
        // Declare and define private member variables.
        int m_nNumTagsToGenerate                                         = 50;
        std::vector<cv::aruco::PredefinedDictionaryType> m_vDictionaries = {cv::aruco::DICT_4X4_50, cv::aruco::DICT_5X5_50, cv::aruco::DICT_6X6_50};

        /******************************************************************************
         * @brief This code will run in a seperate thread. This is the main code.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        void ThreadedContinuousCode()
        {
            // Start thread pool.
            this->RunPool(3);

            // Wait for pool tasks to finish.
            this->JoinPool();

            // Stop threaded code.
            this->RequestStop();
        }

        /******************************************************************************
         * @brief Any highly parallelizable code that can be used in the main thread
         *       goes here.
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

            // Loop through and generate each of the tags.
            for (int i = 0; i < m_nNumTagsToGenerate; ++i)
            {
                GenerateOpenCVArucoMarker(cvDictType, i);
            }
        }

    public:
        // Declare and define public methods and variables.
        ArucoGenerateTags() = default;

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
    // Create Aruco generator threads.
    ArucoGenerateTags TagGenerator1;

    TagGenerator1.Start();
    TagGenerator1.Join();

    return 0;
}
