/******************************************************************************
 * @brief Defines the Bicycle Model class. Used to simply model the kinematics
 *      of a bicycle, which most closely resembles the kinematics of the Mars Rover.
 *      This model can also predict the future state of the rover given a current
 *      state and control input.
 *
 * @file BicycleModel.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-10
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef BICYCLE_MODEL_H
#define BICYCLE_MODEL_H

#include "../../util/NumberOperations.hpp"

/// \cond
#include <chrono>
#include <cmath>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief This class implements the Bicycle Model. This model is used to predict
 *    the future state of the rover given a current state and control input.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2025-01-10
 ******************************************************************************/
class BicycleModel
{
    public:
        /////////////////////////////////////////
        // Declare public structs for this class.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief This struct is used to store the predicted state of the bicycle.
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
         * @brief Construct a new Bicycle Model object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        BicycleModel()
        {
            // Initialize member variables.
            m_dWheelbase       = 0.0;
            m_dXPosition       = 0.0;
            m_dYPosition       = 0.0;
            m_dTheta           = 0.0;
            m_dVelocity        = -1.0;
            m_dSteeringAngle   = 0.0;
            m_tmLastUpdateTime = std::chrono::system_clock::now();
        }

        /******************************************************************************
         * @brief Construct a new Bicycle Model object.
         *
         * @param dWheelbase - The distance between the front and rear axles of the rover.
         * @param dXPosition - The x position of the rover.
         * @param dYPosition - The y position of the rover.
         * @param dTheta - The heading angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        BicycleModel(const double dWheelbase, const double dXPosition, const double dYPosition, const double dTheta)
        {
            // Initialize member variables.
            m_dWheelbase       = dWheelbase;
            m_dXPosition       = dXPosition;
            m_dYPosition       = dYPosition;
            m_dTheta           = dTheta;
            m_dVelocity        = -1.0;
            m_dSteeringAngle   = 0.0;
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
            m_dSteeringAngle   = 0.0;
            m_dVelocity        = -1.0;
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
            m_dSteeringAngle   = 0.0;
            m_dVelocity        = -1.0;
            m_tmLastUpdateTime = std::chrono::system_clock::now();
        }

        /******************************************************************************
         * @brief Update the state of the model, given a new position and heading.
         *      This method will automatically calculate the velocity of the rover
         *      base on the new and current position. The steering angle is also
         *      updated automatically based on the new and current heading.
         *
         * @param dXPosition - The x position of the rover.
         * @param dYPosition - The y position of the rover.
         * @param dTheta - The heading angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void UpdateState(const double dXPosition, const double dYPosition, const double dTheta)
        {
            // Check if this is our first update.
            if (m_dVelocity == -1.0)
            {
                // Set the velocity to zero.
                m_dVelocity = 0.0;
            }
            // Calculate the velocity of the rover as long as the new position is different from the current position.
            else if (dXPosition != m_dXPosition || dYPosition != m_dYPosition)
            {
                // Calculate the velocity of the rover.
                std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
                double dTimeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmLastUpdateTime).count() / 1000.0;
                m_dVelocity         = std::sqrt(std::pow(dXPosition - m_dXPosition, 2) + std::pow(dYPosition - m_dYPosition, 2)) / dTimeElapsed;

                // Update the last update time.
                m_tmLastUpdateTime = tmCurrentTime;
            }

            // Calculate the steering angle of the rover.
            m_dSteeringAngle = (std::atan2(dYPosition - m_dYPosition, dXPosition - m_dXPosition) * 180.0 / M_PI) - m_dTheta;
            // Ensure the steering angle stays within 0-360 degrees.
            m_dSteeringAngle = numops::InputAngleModulus(m_dSteeringAngle, 0.0, 360.0);

            // Update member variables.
            m_dXPosition = dXPosition;
            m_dYPosition = dYPosition;
            m_dTheta     = dTheta;
        }

        /******************************************************************************
         * @brief Accessor for the State private member
         *
         * @param dTimeStep - The time step to predict the future state. How far into the future to predict.
         * @param nNumPredictions - The number of predictions to make.
         * @param vPredictions - The vector of predictions to store the predicted states.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
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
                // Convert theta from degrees to radians for calculation.
                double dThetaRad         = dThetaPredicted * M_PI / 180.0;
                double dSteeringAngleRad = m_dSteeringAngle * M_PI / 180.0;

                // Calculate the new state.
                dXPredicted += m_dVelocity * std::sin(dThetaRad) * dTimeStep;
                dYPredicted += m_dVelocity * std::cos(dThetaRad) * dTimeStep;
                dThetaPredicted += (m_dVelocity / m_dWheelbase) * std::tan(dSteeringAngleRad - M_PI_2) * dTimeStep;

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
         * @brief Mutator for the Wheelbase private member
         *
         * @param dWheelbase - The distance between the front and rear axles of the rover.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetWheelbase(const double dWheelbase) { m_dWheelbase = dWheelbase; }

        /******************************************************************************
         * @brief Mutator for the XPosition private member
         *
         * @param dXPosition - The x position of the rover.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetXPosition(const double dXPosition) { m_dXPosition = dXPosition; }

        /******************************************************************************
         * @brief Mutator for the YPosition private member
         *
         * @param dYPosition - The y position of the rover.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetYPosition(const double dYPosition) { m_dYPosition = dYPosition; }

        /******************************************************************************
         * @brief Mutator for the Theta private member
         *
         * @param dTheta - The heading angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetTheta(const double dTheta) { m_dTheta = dTheta; }

        /******************************************************************************
         * @brief Mutator for the Steering Angle private member
         *
         * @param dSteeringAngle - The steering angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        void SetSteeringAngle(const double dSteeringAngle) { m_dSteeringAngle = dSteeringAngle; }

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Accessor for the Wheelbase private member.
         *
         * @return double - The distance between the front and rear axles of the rover.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetWheelbase() const { return m_dWheelbase; }

        /******************************************************************************
         * @brief Accessor for the XPosition private member.
         *
         * @return double - The x position of the rover.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetXPosition() const { return m_dXPosition; }

        /******************************************************************************
         * @brief Accessor for the YPosition private member.
         *
         * @return double - The y position of the rover.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetYPosition() const { return m_dYPosition; }

        /******************************************************************************
         * @brief Accessor for the Theta private member.
         *
         * @return double - The heading angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetTheta() const { return m_dTheta; }

        /******************************************************************************
         * @brief Accessor for the Velocity private member.
         *
         * @return double - The velocity of the rover.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetVelocity() const { return m_dVelocity; }

        /******************************************************************************
         * @brief Accessor for the Steering Angle private member.
         *
         * @return double - The steering angle of the rover in degrees.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2025-01-10
         ******************************************************************************/
        double GetSteeringAngle() const { return m_dSteeringAngle; }

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        double m_dWheelbase;
        double m_dXPosition;
        double m_dYPosition;
        double m_dTheta;
        double m_dVelocity;
        double m_dSteeringAngle;
        std::chrono::system_clock::time_point m_tmLastUpdateTime;
};
#endif
