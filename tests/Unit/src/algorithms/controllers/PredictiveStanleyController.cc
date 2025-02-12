/******************************************************************************
 * @brief Unit Test Class for the PredictiveStanleyController
 *
 * @file PredictiveStanleyController.cc
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "../../../../../src/algorithms/controllers/PredictiveStanleyController.h"
#include "../../../../../src/util/GeospatialOperations.hpp"
#include "../../../../TestingBase.hh"

/// \cond
#include <gtest/gtest.h>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief Test Class for the PredictiveStanleyController
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
class PredictiveStanleyControllerTests : public TestingBase<PredictiveStanleyControllerTests>
{
    protected:
        // This is where you can declare variables that are used in multiple tests.
        // Just do any setup or teardown in the SetUp and TearDown methods respectively.

    public:
        /******************************************************************************
         * @brief Construct a new PredictiveStanleyControllerTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-11
         ******************************************************************************/
        PredictiveStanleyControllerTests() {}

        /******************************************************************************
         * @brief Destroy the PredictiveStanleyControllerTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-11
         ******************************************************************************/
        ~PredictiveStanleyControllerTests() {}

        /******************************************************************************
         * @brief Setup the PredictiveStanleyControllerTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-11
         ******************************************************************************/
        void TestSetup() override {}

        /******************************************************************************
         * @brief Teardown the PredictiveStanleyControllerTests object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-02-11
         ******************************************************************************/
        void TestTeardown() override {}
};

/******************************************************************************
 * @brief Test the default constructor of PredictiveStanleyController.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, DefaultConstructor)
{
    controllers::PredictiveStanleyController Controller;
    EXPECT_NEAR(Controller.GetControlGain(), constants::STANLEY_CROSSTRACK_CONTROL_GAIN, 0.01);
    EXPECT_NEAR(Controller.GetSteeringAngleLimit(), constants::STANLEY_STEERING_ANGLE_LIMIT, 0.01);
    EXPECT_NEAR(Controller.GetWheelbase(), constants::STANLEY_DIST_TO_FRONT_AXLE, 0.01);
}

/******************************************************************************
 * @brief Test the parameterized constructor of PredictiveStanleyController.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, ParameterizedConstructor)
{
    controllers::PredictiveStanleyController Controller(2.0, 30.0, 1.5, 10, 0.1);
    EXPECT_NEAR(Controller.GetControlGain(), 2.0, 0.01);
    EXPECT_NEAR(Controller.GetSteeringAngleLimit(), 30.0, 0.01);
    EXPECT_NEAR(Controller.GetWheelbase(), 1.5, 0.01);
}

/******************************************************************************
 * @brief Test the SetControlGain method of PredictiveStanleyController.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, SetControlGain)
{
    controllers::PredictiveStanleyController Controller;
    Controller.SetControlGain(3.0);
    EXPECT_NEAR(Controller.GetControlGain(), 3.0, 0.01);
}

/******************************************************************************
 * @brief Test the SetSteeringAngleLimit method of PredictiveStanleyController.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, SetSteeringAngleLimit)
{
    controllers::PredictiveStanleyController Controller;
    Controller.SetSteeringAngleLimit(25.0);
    EXPECT_NEAR(Controller.GetSteeringAngleLimit(), 25.0, 0.01);
}

/******************************************************************************
 * @brief Test the SetWheelbase method of PredictiveStanleyController.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, SetWheelbase)
{
    controllers::PredictiveStanleyController Controller;
    Controller.SetWheelbase(1.8);
    EXPECT_NEAR(Controller.GetWheelbase(), 1.8, 0.01);
}

/******************************************************************************
 * @brief Test the SetReferencePath method of PredictiveStanleyController.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, SetReferencePath)
{
    controllers::PredictiveStanleyController Controller;
    std::vector<geoops::Waypoint> vPath = {geoops::GPSCoordinate(0.0, 0.0), geoops::GPSCoordinate(1.0, 1.0), geoops::GPSCoordinate(2.0, 2.0)};
    Controller.SetReferencePath(vPath);
    EXPECT_EQ(Controller.GetReferencePath().size(), vPath.size());
}

/******************************************************************************
 * @brief Test the SetReferencePath with UTM coordinates method of PredictiveStanleyController.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, SetReferencePathUTM)
{
    controllers::PredictiveStanleyController Controller;
    std::vector<geoops::UTMCoordinate> vPath = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}};
    Controller.SetReferencePath(vPath);
    EXPECT_EQ(Controller.GetReferencePath().size(), vPath.size());
}

/******************************************************************************
 * @brief Test the SetReferencePath with GPS coordinates method of PredictiveStanleyController.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, SetReferencePathGPS)
{
    controllers::PredictiveStanleyController Controller;
    std::vector<geoops::GPSCoordinate> vPath = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}};
    Controller.SetReferencePath(vPath);
    EXPECT_EQ(Controller.GetReferencePath().size(), vPath.size());
}

/******************************************************************************
 * @brief Test the Calculate method of PredictiveStanleyController when no reference path is set.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, CalculateEmptyPath)
{
    controllers::PredictiveStanleyController Controller;
    geoops::RoverPose stPose                                          = {geoops::UTMCoordinate{0.0, 0.0}, 0.0};
    controllers::PredictiveStanleyController::DriveVector driveVector = Controller.Calculate(stPose);
    EXPECT_NEAR(driveVector.dSteeringAngle, 0.0, 0.01);
}

/******************************************************************************
 * @brief Test the Calculate method of PredictiveStanleyController when a reference path is set.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, CalculateFullPath)
{
    controllers::PredictiveStanleyController Controller;
    std::vector<geoops::Waypoint> vPath = {geoops::GPSCoordinate(0.0, 0.0), geoops::GPSCoordinate(1.0, 1.0), geoops::GPSCoordinate(2.0, 2.0)};
    Controller.SetReferencePath(vPath);
    geoops::RoverPose stPose                                          = {geoops::UTMCoordinate{1.0, 1.0}, 0.0};
    controllers::PredictiveStanleyController::DriveVector driveVector = Controller.Calculate(stPose);
    EXPECT_NEAR(driveVector.dSteeringAngle, 100.0, 0.01);
}

/******************************************************************************
 * @brief Test the GetPathTargetIndex method of PredictiveStanleyController.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-02-11
 ******************************************************************************/
TEST_F(PredictiveStanleyControllerTests, GetPathTargetIndex)
{
    controllers::PredictiveStanleyController Controller;
    std::vector<geoops::Waypoint> vPath = {geoops::GPSCoordinate(0.0, 0.0), geoops::GPSCoordinate(1.0, 1.0), geoops::GPSCoordinate(2.0, 2.0)};
    Controller.SetReferencePath(vPath);
    Controller.Calculate({geoops::UTMCoordinate{2.0, 2.0}, 0.0});
    EXPECT_EQ(Controller.GetReferencePathTargetIndex(), 0);
}
