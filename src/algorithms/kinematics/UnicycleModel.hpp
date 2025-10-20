/******************************************************************************
 * @brief Defines the Unicycle Model class. Used to simply model the kinematics
 *      of a unicycle, which most closely resembles the kinematics of the Mars Rover.
 *      This model can also predict the future state of the rover given a current
 *      state and control input.
 *
 * @file UnicycleModel.hpp
 * @author Bailey Schoenike (baileyps03@gmail.com)
 * @date 2025-10-4
 *
 * @copyright Copyright Mars Rover Design Team 2025 - All Rights Reserved
 ******************************************************************************/

#ifndef UNICYCLE_MODEL_H
#define UNICYCLE_MODEL_H

#include "../../util/NumberOperations.hpp"

/// \cond
#include <chrono>
#include <cmath>
#include <vector>

/// \endcond

/******************************************************************************
 * @brief This class implements the Unicycle Model. This model is used to predict
 *    the future state of the rover given a current state and control input.
 *
 *
 * @author Bailey Schoenike (baileyps03@gmail.com)
 * @date 2025-10-4
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
         * @author Bailey Schoenike (baileyps03@gmail.com)
         * @date 2025-10-4
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
         * @author Bailey Schoenike (baileyps03@gmail.com)
         * @date 2025-10-4
         ******************************************************************************/
        UnicycleModel()
        {
            // Initialize member variables.
            m_dXPosition       = 0.0;
            m_dYPosition       = 0.0;
            m_dTheta           = 0.0;
            m_dVelocity        = -1.0;
            m_dAngularVelocity = 0.0;
            m_tmLastUpdateTime = std::chrono::system_clock::now();
        }

        /******************************************************************************
         * @brief Construct a new Unicycle Model object.
         *
         * @param dXPosition - The x position of the rover.
         * @param dYPosition - The y position of the rover.
         * @param dTheta - The heading angle of the rover in degrees.
         *
         * @author Bailey Schoenike (baileyps03@gmail.com)
         * @date 2025-10-4
         ******************************************************************************/
        UnicycleModel(const double dXPosition, const double dYPosition, const double dTheta)
        {
            // Initialize member variables.
            m_dXPosition       = dXPosition;
            m_dYPosition       = dYPosition;
            m_dTheta           = dTheta;
            m_dVelocity        = -1.0;
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
            m_dAngularVelocity = 0.0;
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
            m_dAngularVelocity = 0.0;
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
                double dThetaRad = dThetaPredicted * M_PI / 180.0;

                // Calculate the new state.
                dXPredicted += m_dVelocity * std::sin(dThetaRad) * dTimeStep;
                dYPredicted += m_dVelocity * std::cos(dThetaRad) * dTimeStep;
                dThetaPredicted += (m_dAngularVelocity) *dTimeStep;

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
         * @brief Mutator for the Angular Velocity private member
         *
         * @param dAngularVelocity - The angular velocity of the rover in degrees per second.
         *
         * @author Bailey Schoenike (baileyps03@gmail.com)
         * @date 2025-10-4
         ******************************************************************************/
        void SetAngularVelocity(const double dAngularVelocity) { m_dAngularVelocity = dAngularVelocity; }

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////

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
         * @brief Accessor for the Angular Velocity private member.
         *
         * @return double - The angular velocity of the rover in degrees per second.
         *
         * @author Bailey Schoenike (baileyps03@gmail.com)
         * @date 2025-10-4
         ******************************************************************************/
        double GetAngularVelocity() const { return m_dAngularVelocity; }

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        double m_dXPosition;
        double m_dYPosition;
        double m_dTheta;
        double m_dVelocity;
        double m_dAngularVelocity;
        std::chrono::system_clock::time_point m_tmLastUpdateTime;
};
#endif
