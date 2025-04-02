#include "KalmanFilter.h"

namespace filters
{
    KalmanFilter::XStateSnapshot KalmanFilter::Interpolate(double dRatio, const KalmanFilter::XStateSnapshot& sBefore, const KalmanFilter::XStateSnapshot& sAfter)
    {
        return KalmanFilter::XStateSnapshot{.eiPosition  = sBefore.eiPosition + dRatio * (sAfter.eiPosition - sBefore.eiPosition),
                                            .eiVelocity  = sBefore.eiVelocity + dRatio * (sAfter.eiVelocity - sBefore.eiVelocity),
                                            .eiRotation  = sBefore.eiRotation + dRatio * (sAfter.eiRotation - sBefore.eiRotation),
                                            .tmTimestamp = sBefore.tmTimestamp + (sAfter.tmTimestamp - sBefore.tmTimestamp)};
    }

    KalmanFilter::XStateSnapshot KalmanFilter::GetCurrentState()
    {
        return m_liXStateHistory.back();
    }

    KalmanFilter::XStateSnapshot KalmanFilter::GetInterpolatedHistory(time_point_t tmTimestamp)
    {
        if (m_liXStateHistory.empty())    // panic!
        {}

        if (tmTimestamp < m_liXStateHistory.back().tmTimestamp)
        {
            time_point_t tmFirstBefore, tmFirstAfter;
            // Find the first state before the given timestamp
            auto stdBegin = m_liXStateHistory.rbegin();
            auto stdEnd   = m_liXStateHistory.rend();
            // Iterate from end
            auto stdBefore = std::find(stdBegin, stdEnd, [&tmTimestamp](const XStateSnapshot& stdEntry) { return stdEntry.tmTimestamp < tmTimestamp; });
            if (stdBefore != stdEnd)
            {
                if (stdBefore != stdBegin)
                {
                    // The next state will have been greater than or equal to the given timestamp
                    auto stdAfter = stdBefore - 1;
                    //
                    std::chrono::duration<double> tmDtTotal   = stdAfter->tmTimestamp - stdBefore->tmTimestamp;
                    std::chrono::duration<double> tmDtCurrent = tmTimestamp - stdBefore->tmTimestamp;
                    // Interpolate between the two states
                    double dInterpRatio = tmDtCurrent / tmDtTotal;
                }
            }
            else
            {}
        }
        else
        {
            return m_liXStateHistory.back();
        }
    }

    void KalmanFilter::PredictAccelerometer(Eigen::Vector3d eiAccelerometerOutput, time_point_t tmTimestamp) {}

    void KalmanFilter::PredictGyro(Eigen::Vector3d eiGyroOutput, time_point_t tmTimestamp) {}

    void KalmanFilter::UpdateDiffGPS(const geoops::GPSCoordinate& sDiffGPSCoordinate, time_point_t tmTimestamp) {}

    void KalmanFilter::UpdateHeading(Eigen::Vector3d eiAccOutput, time_point_t tmTimestamp) {}

    void KalmanFilter::SetAccelerometerCovariance(Eigen::Matrix3d eiNewCovariance) {}

    void KalmanFilter::SetGyroCovariance(Eigen::Matrix3d eiNewCovariance) {}

    void KalmanFilter::SetDiffGPSCovariance(Eigen::Matrix3d eiNewCovariance) {}

    void KalmanFilter::SetHeadingCovariance(Eigen::Matrix3d eiNewCovariance) {}
}    // namespace filters
