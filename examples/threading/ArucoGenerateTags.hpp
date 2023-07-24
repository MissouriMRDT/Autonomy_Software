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
#include "../../src/interfaces/AutonomyThreading.hpp"
#include "../opencv/TagGenerator.hpp"

class ArucoGenerateTags : public AutonomyThreading
{
    private:
        // Declare and define private member variables.
        int m_nTagCounter                                 = 0;
        int m_nNumTagsToGenerate                          = 0;
        cv::aruco::PredefinedDictionaryType m_eDictionary = cv::aruco::DICT_4X4_50;

        /******************************************************************************
         * @brief This code will run in a seperate thread.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0723
         ******************************************************************************/
        void ThreadedCode()
        {
            // Generate tag with id from 4x4 50 dictionary.
            GenerateOpenCVArucoMarker(cv::aruco::DICT_4X4_50, m_nTagCounter);

            // Increment tag counter.
            ++m_nTagCounter;

            // Check if we have reached 50 tags.
            if (m_nTagCounter < m_nNumTagsToGenerate)
            {
                // Stop threaded code.
                this->RequestStop();
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
        void SetTagDictionaryType(const cv::aruco::PredefinedDictionaryType eDictionary) { m_eDictionary = eDictionary; }
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

    return 0;
}
