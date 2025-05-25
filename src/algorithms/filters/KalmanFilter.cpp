#include "KalmanFilter.h"

#define unless(content) if (!(content))

namespace filters
{
    /******************************************************************************
     * @brief Linear interpolate between two state snapshots. This function does not clamp.
     *
     * @param dRatio - A value 0.0 to 1.0
     * @param stBefore - Lower bound.
     * @param stAfter - Upper bound.
     * @return KalmanFilter::XStateSnapshot - A mix between the two states.
     *
     * @author Adam
     * @date 2025-05-20
     ******************************************************************************/
    KalmanFilter::XStateSnapshot KalmanFilter::InterpolateXState(double dRatio, const KalmanFilter::XStateSnapshot& stBefore, const KalmanFilter::XStateSnapshot& stAfter)
    {
        dRatio = std::clamp(dRatio, 0.0, 1.0);
        XStateSnapshot stInterpolated;
        stInterpolated.eiPosition  = (1 - dRatio) * stBefore.eiPosition + dRatio * stAfter.eiPosition,
        stInterpolated.eiVelocity  = (1 - dRatio) * stBefore.eiVelocity + dRatio * stAfter.eiVelocity,
        stInterpolated.eiRotation  = (1 - dRatio) * stBefore.eiRotation + dRatio * stAfter.eiRotation,
        stInterpolated.tmTimestamp = stBefore.tmTimestamp + std::chrono::duration_cast<std::chrono::nanoseconds>(dRatio * (stAfter.tmTimestamp - stBefore.tmTimestamp));
        return stInterpolated;
    }

    /******************************************************************************
     * @brief Set the initial guess for the Kalman Filter. The filter cannot make predictions
     * about future state until a UTM coordinate has been provided to serve as the origin.
     *
     * @param stOrigin - The UTM coordinate to use as the origin.
     *
     * @author Adam
     * @date 2025-05-21
     ******************************************************************************/
    void KalmanFilter::SetInitialGuess(const geoops::UTMCoordinate& stOrigin, const double dHeading)
    {
        m_stInitialGuess   = geoops::RoverPose(stOrigin, dHeading);
        m_bHasInitialGuess = true;
    }

    KalmanFilter::XStateSnapshot KalmanFilter::GetCurrentState() const
    {
        // If there's no value, just return some default initialized value like NavigationBoard does.
        if (!m_bHasInitialGuess)
        {
            LOG_WARNING(logging::g_qSharedLogger, "KalmanFilter is has not yet been given an initial guess.");
            return XStateSnapshot();
        }

        return m_liXStateHistory.back();
    }

    geoops::RoverPose KalmanFilter::GetCurrentPose() const
    {
        // If there's no value, just return some default initialized value like NavigationBoard does.
        if (!m_bHasInitialGuess)
        {
            LOG_WARNING(logging::g_qSharedLogger, "KalmanFilter is has not yet been given an initial guess.");
        }

        geoops::UTMCoordinate stReturn = m_stInitialGuess;
        return m_liXStateHistory.back();

        /******************************************************************************
         * @brief Look up a past state estimate at the given time, interpolated between estimates at the
         * nearest recorded times.
         *
         * @param tmTimestamp - The time at which the estimate was made.
         * @return KalmanFilter::XStateSnapshot - An estimate interpolated between recorded values.
         * If tmTimestamp is after the most recent estimate, the most recent estimate is returned.
         * If tmTimestamp is before the oldest estimate, the oldest estimate is returned.
         *
         * @author Adam
         * @date 2025-05-21
         ******************************************************************************/
        KalmanFilter::XStateSnapshot KalmanFilter::GetInterpolatedHistory(time_point_t tmTimestamp) const
        {
            // If there's no value, just return some default initialized value like NavigationBoard does.
            if (!m_bHasInitialGuess)
            {
                LOG_WARNING(logging::g_qSharedLogger, "KalmanFilter is has not yet been given an initial guess.");
                return XStateSnapshot();
            }

            // If given timestamp is after the most recent time, just return the most recent state.
            // TODO: Possibly interpolate this one forwards in the future
            if (tmTimestamp >= m_liXStateHistory.back().tmTimestamp)
            {
                return m_liXStateHistory.back();
            }
            // Otherwise, it is guaranteed that there is at least one state after the given timestamp.

            // Find the first state before the given timestamp.
            time_point_t tmFirstBefore, tmFirstAfter;
            auto stdBegin = m_liXStateHistory.rbegin();
            auto stdEnd   = m_liXStateHistory.rend();
            // Search from end (newest estimates are at end).
            auto stdBefore = std::find(stdBegin, stdEnd, [&tmTimestamp](const XStateSnapshot& stdEntry) { return stdEntry.tmTimestamp < tmTimestamp; });
            // There is a state before the given timestamp:
            if (stdBefore != stdEnd)
            {
                // It is guaranteed that there is at least one state after the given timestamp.
                auto stdAfter = stdBefore - 1;
                //
                std::chrono::duration<double> tmDtCurrent = tmTimestamp - stdBefore->tmTimestamp;
                std::chrono::duration<double> tmDtTotal   = stdAfter->tmTimestamp - stdBefore->tmTimestamp;
                // Interpolate between stBefore and stAfter by dRatio.
                double dRatio = tmDtCurrent / tmDtTotal;
                // Clamp just to be safe.
                dRatio = std::clamp(dRatio, 0.0, 1.0);
                XStateSnapshot stInterpolated;
                stInterpolated.eiPosition  = (1 - dRatio) * stdBefore->eiPosition + dRatio * stdAfter->eiPosition;
                stInterpolated.eiVelocity  = (1 - dRatio) * stdBefore->eiVelocity + dRatio * stdAfter->eiVelocity;
                stInterpolated.eiRotation  = (1 - dRatio) * stdBefore->eiRotation + dRatio * stdAfter->eiRotation;
                stInterpolated.tmTimestamp = tmTimestamp;
                return stInterpolated;
            }
            // There is no state before the given time stamp, so just return the oldest state:
            else
            {
                return m_liXStateHistory.front();
            }
        }

        /******************************************************************************
         * @brief Make prediction of future accelerometer state computed from the state space model.
         * See https://en.wikipedia.org/wiki/Kalman_filter#Predict
         *
         * @param eiAccelerometerOutput - A column vector of accelerometer output in the NED frame [an, ae, ad]
         * @param tmTimestamp - The timestamp of this measurement
         *
         * @author OcelotEmpire (hobbz.pi@gmail.com)
         * @date 2025-04-02
         ******************************************************************************/
        void KalmanFilter::PredictAccelerometer(Eigen::Vector3d eiAccelerometerOutput, time_point_t tmTimestamp)
        {
            XStateSnapshot stNextState{.tmTimestamp = tmTimestamp};
            auto dt = tmTimestamp - m_tmLastAccelerometerUpdate;
            // TODO: Subtract gravity from accelerometer measurement
        }

        void KalmanFilter::PredictGyroscope(Eigen::Vector3d eiGyroscopeOutput, time_point_t tmTimestamp) {}

        // Direct measurement of the state space
        // See https://en.wikipedia.org/wiki/Kalman_filter#Update
        void KalmanFilter::UpdateGPS(Eigen::Vector3d eiGPSOutputNEDFrame, time_point_t tmTimestamp) {}

        void KalmanFilter::UpdateHeading(Eigen::Vector3d eiAccOutput, time_point_t tmTimestamp) {}

        void KalmanFilter::SetAccelerometerCovariance(Eigen::Matrix3d eiNewCovariance) {}

        void KalmanFilter::SetGyroscopeCovariance(Eigen::Matrix3d eiNewCovariance) {}

        void KalmanFilter::SetGPSCovariance(Eigen::Matrix3d eiNewCovariance) {}

        void KalmanFilter::SetHeadingCovariance(Eigen::Matrix3d eiNewCovariance) {}

        // Returns the covariance of process noise from noisy accelerometer input.
        Eigen::Matrix3d KalmanFilter::GetQAccelerometer(double dDt)
        {
            // For an error in acceleration, velocity will vary by 1t and position will vary by (1/2)t^2
            // Rotation is not affected by these variations.
            Eigen::Matrix3d eiG{
                {0.5 * dDt * dDt, 0.5 * dDt * dDt, 0.5 * dDt * dDt},    // Position
                {dDt, dDt, dDt},                                        // Velocity
                {0, 0, 0},                                              // Rotation
            };
            Eigen::Matrix3d eiQ = eiG * m_eiAccelerometerCovariance * m_eiAccelerometerCovariance * eiG.transpose();
            return eiQ;
        }

        // Returns the covariance of process noise from noisy gyroscope input.
        Eigen::Matrix3d KalmanFilter::GetQGyroscope(double dDt)
        {
            Eigen::Matrix3d eiG{{0, 0, 0}, {0, 0, 0}, {dDt, dDt, dDt}};
            Eigen::Matrix3d eiQ = eiG * m_eiGyroscopeCovariance * m_eiGyroscopeCovariance * eiG.transpose();
            return eiQ;
        }

    }    // namespace filters
