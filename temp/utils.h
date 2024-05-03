/******************************************************************************
 * @brief
 *
 * @file utils.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-05-02
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#pragma once

enum Det
{
    tl_x      = 0,
    tl_y      = 1,
    br_x      = 2,
    br_y      = 3,
    score     = 4,
    class_idx = 5
};

struct Detection
{
        cv::Rect bbox;
        float score;
        int class_idx;
};
