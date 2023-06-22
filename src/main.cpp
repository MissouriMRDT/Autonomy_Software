/******************************************************************************
 * @brief Main program file. Sets up classes and runs main program functions.
 *
 * @file main.cpp
 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0620
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../examples/OpenCV/Tag_Generator.h"

/******************************************************************************
 * @brief Autonomy main function.
 *
 * @return int - Exit status number.
 *
 * @author Byrdman32 (eli@byrdneststudios.com), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0620
 ******************************************************************************/
int main()
{

    GenerateOpenCVArucoMarker(cv::aruco::DICT_4X4_50, 1);

    return 0;
}
