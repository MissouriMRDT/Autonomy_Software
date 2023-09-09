/******************************************************************************
 * @brief This util provides an easy way to keep track of iterations per second for
 *      any body of code.
 *
 * @file IPS.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef IPS_HPP
#define IPS_HPP

#include <algorithm>
#include <chrono>
#include <deque>

/******************************************************************************
 * @brief Define and implement the IPS class.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-08-17
 ******************************************************************************/
class IPS
{
    private:
        // Declare private methods and member variables.
        double m_dCurrentIPS;
        double m_dHighestIPS;
        double m_dLowestIPS;
        double m_d1PercentLow;
        std::deque<double> m_dqIPSHistory;
        std::chrono::high_resolution_clock::time_point m_tLastUpdateTime;

        // Define class constants.
        const long unsigned int m_nMaxMetricsHistorySize = 100;

        /******************************************************************************
         * @brief This method is used to calculate the IPS stats. Highest, lowest, and 1
         *      percent low IPS.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-17
         ******************************************************************************/
        void UpdateMetrics()
        {
            // Check highest IPS.
            if (m_dCurrentIPS > m_dHighestIPS)
            {
                // Update new highest.
                m_dHighestIPS = m_dCurrentIPS;
            }
            // Check lowest IPS.
            if (m_dCurrentIPS < m_dLowestIPS)
            {
                // Update new lowest.
                m_dLowestIPS = m_dCurrentIPS;
            }

            // Add current IPS to history.
            m_dqIPSHistory.emplace_back(m_dCurrentIPS);
            // Sort the IPS history vector, this allows us to find the 1% low more easily.
            std::sort(m_dqIPSHistory.begin(), m_dqIPSHistory.end());
            // Throw out oldest element in deque if size is over given number.
            if (m_dqIPSHistory.size() > m_nMaxMetricsHistorySize)
            {
                m_dqIPSHistory.pop_back();
            }
            // Get the index correlating to the IPS number that was lower than 99% of the rest of the IPS history.
            long unsigned int nIndex = static_cast<long unsigned int>(m_dqIPSHistory.size() * 0.01);
            // Check that the index is valid. If history data is too sparse, set 1% low to 0.0.
            m_d1PercentLow = (nIndex < m_dqIPSHistory.size()) ? m_dqIPSHistory[nIndex] : 0.0;
        }

    public:
        // Declare public methods and member variables.
        /******************************************************************************
         * @brief Construct a new IPS object.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-17
         ******************************************************************************/
        IPS()
        {
            // Initialize member variables and objects.
            m_dCurrentIPS  = 0.0;
            m_dHighestIPS  = 0.0;
            m_dLowestIPS   = 9999999;
            m_d1PercentLow = 0.0;
        }

        /******************************************************************************
         * @brief Construct a new IPS object.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-17
         ******************************************************************************/
        ~IPS()
        {
            // Nothing to destroy.
        }

        /******************************************************************************
         * @brief This method is used to update the iterations per second counter and
         *      recalculate all of the IPS metrics.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-17
         ******************************************************************************/
        void Tick()
        {
            // Get the current and elapsed time.
            auto tCurrentTime              = std::chrono::high_resolution_clock::now();
            auto tElapsedTimeSinceLastTick = std::chrono::duration_cast<std::chrono::microseconds>(tCurrentTime - m_tLastUpdateTime).count();

            // Calculate current IPS. 1 second == 1000000 microseconds.
            m_dCurrentIPS = 1e6 / tElapsedTimeSinceLastTick;

            // Calculate IPS overall stats.
            this->UpdateMetrics();

            // Set current time to old.
            m_tLastUpdateTime = tCurrentTime;
        }

        /******************************************************************************
         * @brief Accessor for the Current I P S private member. This method will return the
         *      immediate IPS since the last Tick() call. If called in a loop, this number will
         *      likely jump around greatly. If you want a 'smoother' IPS number then call the
         *      GetAverageIPS() method.
         *
         * @return double - The calculated IPS value since the last iteration.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-17
         ******************************************************************************/
        double GetExactIPS() const
        {
            // Return current iterations per second.
            return m_dCurrentIPS;
        }

        /******************************************************************************
         * @brief Calculates the average iterations per second.
         *
         * @return double - The average iterations per second according to the metrics
         *                  window.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-17
         ******************************************************************************/
        double GetAverageIPS() const
        {
            // Check if there is any history.
            if (m_dqIPSHistory.empty())
            {
                // Return zero IPS average.
                return 0.0;
            }

            // Loop through IPS history and calculate average.
            double dSum = 0.0;
            for (const double dVal : m_dqIPSHistory)
            {
                // Add FPS to total sum.
                dSum += dVal;
            }

            // Return calulated average.
            return dSum / m_dqIPSHistory.size();
        }

        /******************************************************************************
         * @brief Accessor for the Highest I P S private member.
         *
         * @return double - The highest recorded IPS over this objects lifespan.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-17
         ******************************************************************************/
        double GetHighestIPS() const
        {
            // Return the highest iterations per second.
            return m_dHighestIPS;
        }

        /******************************************************************************
         * @brief Accessor for the Lowest I P S private member.
         *
         * @return double - The lowest recorded IPS over this objects lifespan.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-17
         ******************************************************************************/
        double GetLowestIPS() const
        {
            // Return the lowest iterations per second.
            return m_dLowestIPS;
        }

        /******************************************************************************
         * @brief Accessor for the 1PercentLow I P S private member.
         *
         * @return double - The recorded 1% low within the given metrics window size.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-08-17
         ******************************************************************************/
        double Get1PercentLow() const
        {
            // Return the 1% low for IPS.
            return m_d1PercentLow;
        }

        /******************************************************************************
         * @brief Resets all metrics and frame time history.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-09-07
         ******************************************************************************/
        void Reset()
        {
            // Reset member variable.
            m_dCurrentIPS  = 0.0;
            m_dHighestIPS  = 0.0;
            m_dLowestIPS   = 9999999;
            m_d1PercentLow = 0.0;
            // Reset deque.
            m_dqIPSHistory.clear();
        }
};
#endif
