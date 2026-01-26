/******************************************************************************
 * @brief Implements the Extended Kalman Filter class.
 *
 * @file ExtendedKalmanFilter.cpp
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2025-09-27
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#include "ExtendedKalmanFilter.h"
#include "../../AutonomyConstants.h"

#include <GeographicLib/Geodesic.hpp>

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs used to implement the
 *      Extended Kalman Filter.
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2025-09-27
 ******************************************************************************/
namespace filters
{
    /******************************************************************************
     * @brief Construct a new Extended Kalman Filter:: Extended Kalman Filter object.
     *
     *@param stInitPose - The initial GPS and heading of the rover.
     *@param eiAccelCov - The acceleration covariance matrix.
     *@param eiGyroCov -The gyroscope covariance matrix.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-09-30
     ******************************************************************************/
    ExtendedKalmanFilter::ExtendedKalmanFilter(const geoops::RoverPose& stInitPose, const Eigen::Matrix3d& eiAccelCov, const Eigen::Matrix3d& eiGyroCov)
    {
        // Set the covariance matrices.
        m_eiAccelerometerCovariance = eiAccelCov;
        m_eiGyroscopeCovariance     = eiGyroCov;

        // Set the member variables.
        m_dSigmaAccBias  = 0.001;
        m_dSigmaGyroBias = 0.001;
        m_eiGravity      = Eigen::Vector3d(0.0, 0.0, 9.80665);

        // Call method to set the initial guess.
        SetInitialGuess(stInitPose);
    }

    /******************************************************************************
     * @brief This will set the initial guess of the Extended Kalman Filter.
     *
     * @param stInitPose - The initial RoverPose.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-01-25
     ******************************************************************************/
    void ExtendedKalmanFilter::SetInitialGuess(const geoops::RoverPose& stInitPose)
    {
        // Locking while writing.
        std::unique_lock<std::shared_mutex> lkStateWrite(m_muStateMutex);

        // Set origin GPS point.
        m_stOriginGPS = stInitPose.GetGPSCoordinate();

        // Check for valid origin.
        if (std::abs(m_stOriginGPS.dLatitude) > 1e-5)
        {
            m_bOriginSet = true;
        }

        // Update GPS measurement noise.
        geoops::GPSCoordinate stGPS = stInitPose.GetGPSCoordinate();

        // Set horizontal GPS accuracy.
        m_dSigmaGPSHor = (stGPS.d2DAccuracy > 0.0) ? stGPS.d2DAccuracy : 5.0;

        // Set vertical GPS accuracy.
        double d3DAcc  = stGPS.d3DAccuracy;
        m_dSigmaGPSVer = std::sqrt(std::max(0.1, (d3DAcc * d3DAcc) - (m_dSigmaGPSHor * m_dSigmaGPSHor)));

        // Initialize state vector (15x15 matrix).
        m_stCurrentState.eiPosition = Eigen::Vector3d::Zero();

        // Set the orientation.
        m_stCurrentState.eiOrientation = Eigen::AngleAxisd(stInitPose.GetCompassHeading() * M_PI / 180.0, Eigen::Vector3d::UnitZ());

        // Set velocity.
        m_stCurrentState.eiVelocity = Eigen::Vector3d::Zero();

        // Set acceleration and gyro biases.
        m_stCurrentState.eiAccelBias = Eigen::Vector3d::Zero();
        m_stCurrentState.eiGyroBias  = Eigen::Vector3d::Zero();

        // Set the timestamp.
        m_stCurrentState.tmTimestamp = std::chrono::system_clock::now();

        // Initialize P Matrix (error-state covariance).
        m_eiErrorStateCov = Eigen::Matrix<double, 15, 15>::Zero();

        // Set the position uncertainty (East/North/Up).
        m_eiErrorStateCov(0, 0) = m_dSigmaGPSHor * m_dSigmaGPSHor;    // East
        m_eiErrorStateCov(1, 1) = m_dSigmaGPSHor * m_dSigmaGPSHor;    // North
        m_eiErrorStateCov(2, 2) = m_dSigmaGPSVer * m_dSigmaGPSVer;    // Up

        // Set the orientation uncertainty.
        m_eiErrorStateCov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (M_PI * M_PI / 324.0);    // (10pi/180)^2

        // Set the velocity uncertainty.
        m_eiErrorStateCov.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1.0;

        // Set bias uncertainties.
        m_eiErrorStateCov.block<3, 3>(9, 9)   = Eigen::Matrix3d::Identity() * 0.01;
        m_eiErrorStateCov.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.01;

        // Set IMU update timestamp.
        m_tmLastIMUUpdate = std::chrono::system_clock::now();

        // Confirm initial guess set.
        m_bHasInitialGuess = true;
    }

    /******************************************************************************
     * @brief This will set the GPS data noise.
     *
     * @param stCoord - The GPS coordinate.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    void ExtendedKalmanFilter::SetGPSNoise(const geoops::GPSCoordinate& stCoord)
    {
        // Clear existing covariance.
        m_eiGPSCovariance.setZero();

        // Setting horizontal and vertical accuracies.
        m_dSigmaGPSHor = (stCoord.d2DAccuracy > 0.0) ? stCoord.d2DAccuracy : 1.0;
        double d3DAcc  = stCoord.d3DAccuracy;
        m_dSigmaGPSVer = std::sqrt(std::max(0.0, d3DAcc * d3DAcc - m_dSigmaGPSHor * m_dSigmaGPSHor));

        // Updating R matrix.
        m_eiGPSCovariance = Eigen::Matrix3d::Zero();

        // Horizontal noise (X = East/West, Y = North/South).
        m_eiGPSCovariance(0, 0) = m_dSigmaGPSHor * m_dSigmaGPSHor;    // variance in X
        m_eiGPSCovariance(1, 1) = m_dSigmaGPSHor * m_dSigmaGPSHor;    // variance in Y

        // Vertical noise (Z = Up/Down).
        m_eiGPSCovariance(2, 2) = m_dSigmaGPSVer * m_dSigmaGPSVer;    // variance in Z
    }

    /******************************************************************************
     * @brief This will set the compass noise.
     *
     * @param dSigmaDeg - The standard deviation of the compass in degrees.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-01-15
     ******************************************************************************/
    void ExtendedKalmanFilter::SetCompassNoise(double dSigmaDeg)
    {
        // Ensure we don't set it to 0.
        if (dSigmaDeg < 0.1)
        {
            dSigmaDeg = 0.1;
        }

        // Convert to radians and store.
        m_dSigmaYaw = dSigmaDeg * M_PI / 180.0;
    }

    /******************************************************************************
     * @brief The main predict/estimate step for EKF. Integrates IMU data to predict state.
     *
     * @param eiAccelMeas - The accelerometer reading.
     * @param eiGyroMeas - The gyrometer reading.
     * @param tmTimestamp - The timestamp that the prediction has occurred.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-28
     ******************************************************************************/
    void ExtendedKalmanFilter::Predict(Eigen::Vector3d& eiAccelMeas, Eigen::Vector3d& eiGyroMeas, std::chrono::system_clock::time_point tmTimestamp)
    {
        // Must have an initial guess.
        if (!m_bHasInitialGuess)
        {
            return;
        }

        double dt         = std::chrono::duration<double>(tmTimestamp - m_tmLastIMUUpdate).count();
        m_tmLastIMUUpdate = tmTimestamp;

        // Locking while reading.
        std::shared_lock<std::shared_mutex> lkStateRead(m_muStateMutex);

        // Accelerometer and gyrometer bias removal.
        Eigen::Vector3d eiAcc  = eiAccelMeas - m_stCurrentState.eiAccelBias;
        Eigen::Vector3d eiGyro = eiGyroMeas - m_stCurrentState.eiGyroBias;

        // Unlocking after reading.
        lkStateRead.unlock();

        // Updating orientation.
        Eigen::Vector3d eiOmega = eiGyro * dt;
        Eigen::Quaterniond eiDq;

        if (eiOmega.norm() > 1e-8)
        {
            eiDq = Eigen::Quaterniond(Eigen::AngleAxisd(eiOmega.norm(), eiOmega.normalized()));
        }

        // Identity quaternion.
        else
        {
            eiDq = Eigen::Quaterniond::Identity();
        }

        // Locking while writing.
        std::unique_lock<std::shared_mutex> lkStateWrite(m_muStateMutex);

        Eigen::Matrix3d eiROld         = m_stCurrentState.eiOrientation.toRotationMatrix();
        m_stCurrentState.eiOrientation = (m_stCurrentState.eiOrientation * eiDq).normalized();

        // Acceleration in the world frame (accounts for gravity).
        Eigen::Vector3d eiAccWorldFrame = (eiROld * eiAcc) - m_eiGravity;

        // Update velocity and position.
        m_stCurrentState.eiVelocity += eiAccWorldFrame * dt;
        m_stCurrentState.eiPosition += (m_stCurrentState.eiVelocity * dt) + ((eiAccWorldFrame * dt * dt) / 2.0);

        // Unlocking after writing.
        lkStateWrite.unlock();

        // Covariance update.
        Eigen::Matrix<double, 15, 15> eiF = Eigen::Matrix<double, 15, 15>::Zero();

        eiF.block<3, 3>(0, 6)             = Eigen::Matrix3d::Identity();
        eiF.block<3, 3>(3, 3)             = -MakeSkewSymmetricMatrix(eiGyro);
        eiF.block<3, 3>(3, 12)            = -Eigen::Matrix3d::Identity();
        eiF.block<3, 3>(6, 3)             = -eiROld * MakeSkewSymmetricMatrix(eiAcc);
        eiF.block<3, 3>(6, 9)             = -eiROld;

        // Discretize F matrix.
        Eigen::Matrix<double, 15, 15> eiFd = Eigen::Matrix<double, 15, 15>::Identity() + (eiF * dt);

        // Process noise Q.
        Eigen::Matrix<double, 15, 15> eiQ = Eigen::Matrix<double, 15, 15>::Zero();
        eiQ.block<3, 3>(3, 3)             = m_eiGyroscopeCovariance * dt * dt;
        eiQ.block<3, 3>(6, 6)             = m_eiAccelerometerCovariance * dt * dt;
        eiQ.block<3, 3>(9, 9)             = (m_dSigmaAccBias * m_dSigmaAccBias * dt) * Eigen::Matrix3d::Identity();
        eiQ.block<3, 3>(12, 12)           = (m_dSigmaGyroBias * m_dSigmaGyroBias * dt) * Eigen::Matrix3d::Identity();

        // Error state covariance update.
        m_eiErrorStateCov = eiFd * m_eiErrorStateCov * eiFd.transpose() + eiQ;
    }

    /******************************************************************************
     * @brief This will update the GPS noise.
     *
     * @param stCoord - The GPS coordinate from the RoverPose.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-21
     ******************************************************************************/
    void ExtendedKalmanFilter::UpdateGPS(const geoops::GPSCoordinate& stCoord)
    {
        //   Check if there is an initial guess set.
        if (!m_bHasInitialGuess)
        {
            return;
        }

        // Check if there is an origin set.
        if (!m_bOriginSet)
        {
            m_stOriginGPS = stCoord;
            m_bOriginSet  = true;
            return;
        }

        // Convert GPS to ENU.
        Eigen::Vector3d eiZ = ConvertGPSToENU(stCoord);

        // Build measurement noise matrix (R) (accuracy may change after SetGPS()).
        SetGPSNoise(stCoord);

        Eigen::Matrix3d eiR = Eigen::Matrix3d::Zero();
        eiR(0, 0)           = m_dSigmaGPSHor * m_dSigmaGPSHor;
        eiR(1, 1)           = m_dSigmaGPSHor * m_dSigmaGPSHor;
        eiR(2, 2)           = m_dSigmaGPSVer * m_dSigmaGPSVer;

        // Locking while reading.
        std::shared_lock<std::shared_mutex> lkStateRead(m_muStateMutex);

        // Predicted position and innovation.
        Eigen::Vector3d eiXpred = m_stCurrentState.eiPosition;    // From RoverPose
        Eigen::Vector3d eiY     = eiZ - eiXpred;

        // Unlocking after reading.
        lkStateRead.unlock();

        // H matrix.
        Eigen::Matrix<double, 3, 15> eiH = Eigen::Matrix<double, 3, 15>::Zero();
        eiH.block<3, 3>(0, 0)            = Eigen::Matrix3d::Identity();

        // S matrix.
        Eigen::Matrix3d eiS = m_eiErrorStateCov.block<3, 3>(0, 0) + eiR;

        // Mahalanobis Distance Gating for outlier rejection.
        double dMahalanobis = eiY.transpose() * eiS.inverse() * eiY;

        if (dMahalanobis > 11.34)
        {
            return;    // Reject update.
        }

        // Kalman gain (K).
        Eigen::Matrix<double, 15, 3> eiK = m_eiErrorStateCov.block<15, 3>(0, 0) * eiS.inverse();

        // Full state correction dx = K * y.
        Eigen::Matrix<double, 15, 1> eiDx = eiK * eiY;

        // Locking while writing.
        std::unique_lock<std::shared_mutex> lkStateWrite(m_muStateMutex);

        // Apply corrections to nominal state.
        // Position update.
        m_stCurrentState.eiPosition += eiDx.block<3, 1>(0, 0);

        // Orientation update.
        Eigen::Vector3d eiTheta = eiDx.block<3, 1>(3, 0);
        // Small-angle quaternion: q_delta ~= [1, 0.5*delta_theta]
        Eigen::Quaterniond eiDq;
        eiDq.w() = 1.0;
        eiDq.x() = 0.5 * eiTheta.x();
        eiDq.y() = 0.5 * eiTheta.y();
        eiDq.z() = 0.5 * eiTheta.z();
        eiDq.normalize();
        m_stCurrentState.eiOrientation = (m_stCurrentState.eiOrientation * eiDq).normalized();

        // Velocity update.
        m_stCurrentState.eiVelocity += eiDx.block<3, 1>(6, 0);

        // Accel and gyro bias update.
        m_stCurrentState.eiAccelBias += eiDx.block<3, 1>(9, 0);
        m_stCurrentState.eiGyroBias += eiDx.block<3, 1>(12, 0);

        // Unlocking after writing.
        lkStateWrite.unlock();

        // Covariance update: P_new = (I - K H) P (I - K H)^T + K R K^T.
        Eigen::Matrix<double, 15, 15> eiI    = Eigen::Matrix<double, 15, 15>::Identity();
        Eigen::Matrix<double, 15, 15> eiImKH = eiI - (eiK * eiH);

        m_eiErrorStateCov                    = eiImKH * m_eiErrorStateCov * eiImKH.transpose() + (eiK * eiR * eiK.transpose());
    }

    /******************************************************************************
     * @brief This will update our compass, or heading.
     *
     * @param dHeading - The rover's current heading.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-01-12
     ******************************************************************************/
    void ExtendedKalmanFilter::UpdateCompass(const double dHeading)
    {
        if (!m_bHasInitialGuess)
        {
            return;
        }

        // Convert measurement to ENU frame.
        double dYawMeas = (90.0 - dHeading) * M_PI / 180.0;

        // Normalize.
        dYawMeas = std::atan2(std::sin(dYawMeas), std::cos(dYawMeas));

        // Locking while reading.
        std::shared_lock<std::shared_mutex> lkStateRead(m_muStateMutex);

        // Extract predicted yaw from state.
        Eigen::Quaterniond q = m_stCurrentState.eiOrientation;
        double dYawPred      = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        // Unlocking after reading.
        lkStateRead.unlock();

        // Calculate Innovation / Residual (y).
        double dResidual = dYawMeas - dYawPred;

        // Handle angle wrapping.
        while (dResidual > M_PI)
        {
            dResidual -= 2.0 * M_PI;
        }

        while (dResidual < -M_PI)
        {
            dResidual += 2.0 * M_PI;
        }

        // Set up measurement noise (R).
        double dR = m_dSigmaYaw * m_dSigmaYaw;

        // Build observation Jacobian (H).
        Eigen::Matrix<double, 1, 15> eiH = Eigen::Matrix<double, 1, 15>::Zero();
        eiH(0, 5)                        = 1.0;

        // Calculate innovation covariance (S).
        double dS = m_eiErrorStateCov(5, 5) + dR;

        // Mahalanobis Gate.
        double dMahalanobis = (dResidual * dResidual) / dS;
        if (dMahalanobis > 6.63)
        {
            return;    // Reject magnetic anomaly.
        }

        // Calculate Kalman gain (K).
        Eigen::VectorXd eiK = m_eiErrorStateCov.col(5) / dS;

        // Compute correction (dx).
        Eigen::VectorXd eiDx = eiK * dResidual;

        // Locking while writing.
        std::unique_lock<std::shared_mutex> lkStateWrite(m_muStateMutex);

        // Apply state corrections.
        // Position update.
        m_stCurrentState.eiPosition += eiDx.block<3, 1>(0, 0);

        // Velocity update.
        m_stCurrentState.eiVelocity += eiDx.block<3, 1>(6, 0);

        // Accel and gyro bias updates.
        m_stCurrentState.eiAccelBias += eiDx.block<3, 1>(9, 0);
        m_stCurrentState.eiGyroBias += eiDx.block<3, 1>(12, 0);

        // Orientation update.
        Eigen::Vector3d eiThetaErr = eiDx.block<3, 1>(3, 0);
        Eigen::Quaterniond eiDq;
        eiDq.w() = 1.0;
        eiDq.x() = 0.5 * eiThetaErr.x();
        eiDq.y() = 0.5 * eiThetaErr.y();
        eiDq.z() = 0.5 * eiThetaErr.z();
        eiDq.normalize();

        m_stCurrentState.eiOrientation = (m_stCurrentState.eiOrientation * eiDq).normalized();

        // Unlocking after writing.
        lkStateWrite.unlock();

        // Update covariance matrix (P).
        Eigen::Matrix<double, 15, 15> eiI    = Eigen::Matrix<double, 15, 15>::Identity();
        Eigen::Matrix<double, 15, 15> eiImKH = eiI - (eiK * eiH);

        m_eiErrorStateCov                    = eiImKH * m_eiErrorStateCov * eiImKH.transpose() + eiK * dR * eiK.transpose();
    }

    /******************************************************************************
     * @brief This will convert a GPS coordinate into ENU.
     *
     * @param stCoord - The GPS coordinate.
     * @return Eigen::Vector3d - The ENU vector.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-22
     ******************************************************************************/
    Eigen::Vector3d ExtendedKalmanFilter::ConvertGPSToENU(const geoops::GPSCoordinate& stCoord)
    {
        // If we haven't set an origin, use the first point we see or the one passed in constructor.
        if (!m_bOriginSet)
        {
            m_stOriginGPS = stCoord;
            m_bOriginSet  = true;
            return Eigen::Vector3d::Zero();
        }

        double dEarthRadius = 6378137.0;

        double dLat         = (stCoord.dLatitude - m_stOriginGPS.dLatitude) * M_PI / 180.0;
        double dLon         = (stCoord.dLongitude - m_stOriginGPS.dLongitude) * M_PI / 180.0;
        double avgLat       = (stCoord.dLatitude + m_stOriginGPS.dLatitude) * 0.5 * M_PI / 180.0;

        // Converting to ENU (East, North, Up)
        double dx = dEarthRadius * dLon * cos(avgLat);              // East
        double dy = dEarthRadius * dLat;                            // North
        double dz = stCoord.dAltitude - m_stOriginGPS.dAltitude;    // Up

        return Eigen::Vector3d(dx, dy, dz);
    }

    /******************************************************************************
     * @brief This will convert ENU (East/North/Up) into GPS. Transforms from local to global frame.
     *
     * @param eiPosition - The current ENU position of the rover.
     * @return Eigen::Vector3d - The vector of the GPS position of the rover.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-01-25
     ******************************************************************************/
    geoops::GPSCoordinate ExtendedKalmanFilter::ConvertENUToGPS(const Eigen::Vector3d& eiPosition) const
    {
        geoops::GPSCoordinate stResult;

        // If origin isn't set, we can't convert. Return default/empty.
        if (!m_bOriginSet)
        {
            return m_stOriginGPS;
        }

        // Calculate latitude.
        double dLatOffsetDeg = (eiPosition.y() / GeographicLib::Geodesic::WGS84().EquatorialRadius()) * 180.0 / M_PI;
        stResult.dLatitude   = m_stOriginGPS.dLatitude + dLatOffsetDeg;

        // Calculate longitude.
        double dCosLat       = std::cos(m_stOriginGPS.dLatitude * (M_PI / 180.0));
        double dLonOffsetDeg = (eiPosition.x() / (GeographicLib::Geodesic::WGS84().EquatorialRadius() * dCosLat)) * 180.0 / M_PI;
        stResult.dLongitude  = m_stOriginGPS.dLongitude + dLonOffsetDeg;

        // Calculate altitude.
        stResult.dAltitude = m_stOriginGPS.dAltitude + eiPosition.z();

        return stResult;
    }

    /******************************************************************************
     * @brief Converts a RoverPose to orientation quaternion.
     *
     * @param stPose - The current RoverPose.
     * @param eiOrientation - The orientation quaternion.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    void ExtendedKalmanFilter::RoverPoseToOrientation(const geoops::RoverPose& stPose, Eigen::Quaterniond& eiOrientation) const
    {
        // Locking while writing.
        std::unique_lock<std::shared_mutex> lkStateWrite(m_muStateMutex);

        //  Convert heading to orientation quaternion
        double dHeading = stPose.GetCompassHeading();
        eiOrientation   = Eigen::AngleAxisd(dHeading, Eigen::Vector3d::UnitZ());

        // Unlocking after writing.
        lkStateWrite.unlock();
    }

    /******************************************************************************
     * @brief Converts a RoverPose to position vector.
     *
     * @param stPose - The current RoverPose.
     * @param eiPosition - The position vector.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-10-03
     ******************************************************************************/
    void ExtendedKalmanFilter::RoverPoseToGPS(const geoops::RoverPose& stPose, Eigen::Vector3d& eiPosition) const
    {
        // Locking while writing.
        std::unique_lock<std::shared_mutex> lkStateWrite(m_muStateMutex);

        // Convert GPSCoordinate to position vector
        eiPosition(0) = stPose.GetGPSCoordinate().dLatitude;
        eiPosition(1) = stPose.GetGPSCoordinate().dLongitude;
        eiPosition(2) = stPose.GetGPSCoordinate().dAltitude;

        // Unlocking after writing.
        lkStateWrite.unlock();
    }

    /******************************************************************************
     * @brief This method will take a vector as an input and output a skew-symmetric matrix.
     *
     * @param eiVec - The input vector (can be any vector)
     * @return Eigen::Matrix3d - The skew symmetric matrix.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-11-24
     ******************************************************************************/
    Eigen::Matrix3d ExtendedKalmanFilter::MakeSkewSymmetricMatrix(const Eigen::Vector3d& eiVec)
    {
        Eigen::Matrix3d eiSkew;
        // I promise you that this looks prettier before the auto-format
        eiSkew << 0, -eiVec.z(), eiVec.y(), eiVec.z(), 0, -eiVec.x(), -eiVec.y(), eiVec.x(), 0;
        return eiSkew;
    }

    /******************************************************************************
     * @brief Returns the current state snapshot.
     *
     * @return const ExtendedKalmanFilter::XStateSnapshot& - The current state snapshot.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-11-26
     ******************************************************************************/
    ExtendedKalmanFilter::XStateSnapshot ExtendedKalmanFilter::GetCurrentState() const
    {
        return m_stCurrentState;
    }

    /******************************************************************************
     * @brief Returns the estimated RoverPose based on the current state, which includes position and heading.
     *
     * @return geoops::RoverPose - The estimated RoverPose based on the current state.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-01-25
     ******************************************************************************/
    geoops::RoverPose ExtendedKalmanFilter::GetEstimatedRoverPose() const
    {
        // Locking while reading.
        std::shared_lock<std::shared_mutex> lkStateRead(m_muStateMutex);

        // Extract position. (ENU -> GPS)
        geoops::GPSCoordinate stEstimatedGPS = ConvertENUToGPS(m_stCurrentState.eiPosition);

        // Extract heading. (Quaternion -> Yaw)
        Eigen::Vector3d eiEuler = m_stCurrentState.eiOrientation.toRotationMatrix().eulerAngles(0, 1, 2);

        // Normalize heading to compass standard and convert yaw to degrees.
        double dCompassHeading = 90.0 - (eiEuler.z() * 180.0 / M_PI);

        // Normalize to [0, 360).
        dCompassHeading = std::fmod(dCompassHeading, 360.0);
        if (dCompassHeading < 0)
        {
            dCompassHeading += 360.0;
        }

        // Construct and return.
        return geoops::RoverPose(stEstimatedGPS, dCompassHeading);
    }

    /******************************************************************************
     * @brief Destroy the Extended Kalman Filter:: Extended Kalman Filter object.
     *
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2025-11-26
     ******************************************************************************/
    ExtendedKalmanFilter::~ExtendedKalmanFilter()
    {
        // Nothing yet
    }

}    // namespace filters
