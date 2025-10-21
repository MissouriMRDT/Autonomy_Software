/******************************************************************************
 * @brief Defines the Unicycle Model class. Used to model the kinematics
 *      of a unicycle-style rover which can perform point turns. The model can
 *      also predict future states given the most recently observed linear and
 *      angular velocities.
 *
 * @file UnicycleModel.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-10
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef UNICYCLE_MODEL_H
#define UNICYCLE_MODEL_H

#include "../../util/NumberOperations.hpp"

/// \cond
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief This class implements the Unicycle Model. This model is used to predict
 *    the future state of the rover given a current state and inferred control inputs.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-10
 ******************************************************************************/
class UnicycleModel
{
    public:
        /////////////////////////////////////////
        // Declare public structs for this class.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief This struct is used to store the predicted state of the unicycle.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        struct Prediction
        {
            public:
                double dXPosition;
                double dYPosition;
                double dTheta;
        };

        /////////////////////////////////////////
        // Declare public class methods.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Construct a new Unicycle Model object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        UnicycleModel()
        {
            // Initialize member variables.
            m_dWheelbase       = 0.0;
            m_dXPosition       = 0.0;
            m_dYPosition       = 0.0;
            m_dTheta           = 0.0;
            m_dVelocity        = 0.0;
            m_dAngularVelocity = 0.0;
            m_tmLastUpdateTime = std::chrono::system_clock::now();
        }

        /******************************************************************************
         * @brief Construct a new Unicycle Model object.
         *
         * @param dWheelbase - The (optional) distance between the front and rear axles of the rover (kept for compatibility).
         * @param dXPosition - The x position of the rover.
         * @param dYPosition - The y position of the rover.
         * @param dTheta - The heading angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        UnicycleModel(const double dWheelbase, const double dXPosition, const double dYPosition, const double dTheta)
        {
            // Initialize member variables.
            m_dWheelbase       = dWheelbase;
            m_dXPosition       = dXPosition;
            m_dYPosition       = dYPosition;
            m_dTheta           = dTheta;
            m_dVelocity        = 0.0;
            m_dAngularVelocity = 0.0;
            m_tmLastUpdateTime = std::chrono::system_clock::now();
        }

        /******************************************************************************
         * @brief Resets the state of the model to a new position and heading.
         *
         * @param dXPosition - The x position of the rover.
         * @param dYPosition - The y position of the rover.
         * @param dTheta - The heading angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void ResetState(const double dXPosition, const double dYPosition, const double dTheta)
        {
            // Update member variables.
            m_dXPosition       = dXPosition;
            m_dYPosition       = dYPosition;
            m_dTheta           = dTheta;
            m_dVelocity        = 0.0;
            m_dAngularVelocity = 0.0;
            m_tmLastUpdateTime = std::chrono::system_clock::now();
        }

        /******************************************************************************
         * @brief Resets the state of the model to a default state.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void ResetState()
        {
            // Update member variables.
            m_dXPosition       = 0.0;
            m_dYPosition       = 0.0;
            m_dTheta           = 0.0;
            m_dVelocity        = 0.0;
            m_dAngularVelocity = 0.0;
            m_tmLastUpdateTime = std::chrono::system_clock::now();
        }

        /******************************************************************************
         * @brief Update the state of the model, given a new position and heading.
         *      This method will automatically calculate the linear velocity and angular
         *      velocity of the rover based on the new and current position/heading.
         *
         * @param dXPosition - The x position of the rover.
         * @param dYPosition - The y position of the rover.
         * @param dTheta - The heading angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void UpdateState(const double dXPosition, const double dYPosition, const double dTheta)
        {
            // Calculate elapsed time in seconds but protect against extremely small dt.
            std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
            double dTimeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmLastUpdateTime).count() / 1000.0;
            if (dTimeElapsed < 1e-4)
            {
                dTimeElapsed = 1e-4;    // prevent division by (near) zero and spurious velocities
            }

            // If our current state is zero (first update), just set the state without computing velocities.
            if (m_dXPosition == 0.0 && m_dYPosition == 0.0 && m_dTheta == 0.0)
            {
                m_dXPosition       = dXPosition;
                m_dYPosition       = dYPosition;
                m_dTheta           = numops::InputAngleModulus(dTheta, 0.0, 360.0);
                m_tmLastUpdateTime = tmCurrentTime;

                return;
            }

            // Compute linear velocity from position change.
            double dDist = std::sqrt(std::pow(dXPosition - m_dXPosition, 2) + std::pow(dYPosition - m_dYPosition, 2));
            m_dVelocity  = dDist / dTimeElapsed;

            // Compute angular velocity from change in heading (shortest angle) in degrees/sec.
            double dAngleDiff  = numops::AngularDifference(dTheta, m_dTheta);    // returns signed smallest difference
            m_dAngularVelocity = dAngleDiff / dTimeElapsed;

            // Update last update time and state.
            m_tmLastUpdateTime = tmCurrentTime;
            m_dXPosition       = dXPosition;
            m_dYPosition       = dYPosition;
            m_dTheta           = numops::InputAngleModulus(dTheta, 0.0, 360.0);
        }

        /******************************************************************************
         * @brief Accessor for the State private member
         *
         * @param dTimeStep - The time step to predict the future state. How far into the future to predict.
         * @param nNumPredictions - The number of predictions to make.
         * @param vPredictions - The vector of predictions to store the predicted states.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void Predict(const double dTimeStep, const int nNumPredictions, std::vector<Prediction>& vPredictions)
        {
            // Start from the current state.
            double dXPredicted     = m_dXPosition;
            double dYPredicted     = m_dYPosition;
            double dThetaPredicted = m_dTheta;

            // Perform prediction for a specified number of time steps.
            for (int nIter = 0; nIter < nNumPredictions; ++nIter)
            {
                // Convert theta from degrees to radians for position integration.
                double dThetaRad = dThetaPredicted * M_PI / 180.0;

                // Advance the state using unicycle kinematics:
                // x_dot = v * sin(theta) ; y_dot = v * cos(theta) ; theta_dot = omega (degrees/sec)
                dXPredicted += m_dVelocity * std::sin(dThetaRad) * dTimeStep;
                dYPredicted += m_dVelocity * std::cos(dThetaRad) * dTimeStep;
                dThetaPredicted += m_dAngularVelocity * dTimeStep;

                // Ensure theta stays within 0-360 degrees.
                dThetaPredicted = numops::InputAngleModulus(dThetaPredicted, 0.0, 360.0);

                // Store the new state.
                Prediction stPrediction{dXPredicted, dYPredicted, dThetaPredicted};
                vPredictions.push_back(stPrediction);
            }
        }

        /////////////////////////////////////////
        // Setters.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Mutator for the Wheelbase private member (kept for compatibility).
         *
         * @param dWheelbase - The distance between the front and rear axles of the rover.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetWheelbase(const double dWheelbase) { m_dWheelbase = dWheelbase; }

        /******************************************************************************
         * @brief Mutator for the XPosition private member
         *
         * @param dXPosition - The x position of the rover.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetXPosition(const double dXPosition) { m_dXPosition = dXPosition; }

        /******************************************************************************
         * @brief Mutator for the YPosition private member
         *
         * @param dYPosition - The y position of the rover.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetYPosition(const double dYPosition) { m_dYPosition = dYPosition; }

        /******************************************************************************
         * @brief Mutator for the Theta private member
         *
         * @param dTheta - The heading angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetTheta(const double dTheta) { m_dTheta = numops::InputAngleModulus(dTheta, 0.0, 360.0); }

        /******************************************************************************
         * @brief Mutator for the linear velocity private member
         *
         * @param dVelocity - The linear velocity of the rover.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetVelocity(const double dVelocity) { m_dVelocity = dVelocity; }

        /******************************************************************************
         * @brief Mutator for the angular velocity private member
         *
         * @param dAngularVelocity - The angular velocity of the rover (deg/sec).
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetAngularVelocity(const double dAngularVelocity) { m_dAngularVelocity = dAngularVelocity; }

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Accessor for the Wheelbase private member.
         *
         * @return double - The distance between the front and rear axles of the rover.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetWheelbase() const { return m_dWheelbase; }

        /******************************************************************************
         * @brief Accessor for the XPosition private member.
         *
         * @return double - The x position of the rover.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetXPosition() const { return m_dXPosition; }

        /******************************************************************************
         * @brief Accessor for the YPosition private member.
         *
         * @return double - The y position of the rover.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetYPosition() const { return m_dYPosition; }

        /******************************************************************************
         * @brief Accessor for the Theta private member.
         *
         * @return double - The heading angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetTheta() const { return m_dTheta; }

        /******************************************************************************
         * @brief Accessor for the Velocity private member.
         *
         * @return double - The linear velocity of the rover.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetVelocity() const { return m_dVelocity; }

        /******************************************************************************
         * @brief Accessor for the Angular Velocity private member.
         *
         * @return double - The angular velocity of the rover in degrees/sec.
         *
         * @author clayjay3 (claytonraycow3@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetAngularVelocity() const { return m_dAngularVelocity; }

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        double m_dWheelbase;
        double m_dXPosition;
        double m_dYPosition;
        double m_dTheta;              // degrees 0-360
        double m_dVelocity;           // meters/sec
        double m_dAngularVelocity;    // degrees/sec
        std::chrono::system_clock::time_point m_tmLastUpdateTime;
};
#endif
