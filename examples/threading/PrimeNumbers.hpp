/******************************************************************************
 * @brief Example file that creates multiple threads that calculate a random amount
 *      of prime numbers.
 *
 * @file PrimeNumbers.hpp
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0722
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <iostream>
#include <vector>

#include "../../src/interfaces/AutonomyThread.hpp"
#include "../../src/util/ExampleChecker.h"

/******************************************************************************
 * @brief This class creates a thread for calculating N prime numbers.
 *     This is an example class demonstrating the use of the threading interface.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-28
 ******************************************************************************/
class PrimeCalculatorThread : public AutonomyThread<void>
{
    private:
        // Declare and define private methods and variables.
        std::vector<int> m_vThreadPrimes;
        int m_nCount        = 10;
        int m_nCurrentCount = 2;

        /******************************************************************************
         * @brief Check if a number if prime.
         *
         * @param num - The number to check.
         * @return true - The number is prime.
         * @return false - The number is not prime.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        bool IsPrime(int& nNum)
        {
            if (nNum <= 1)
            {
                return false;
            }
            for (int i = 2; i * i <= nNum; ++i)
            {
                if (nNum % i == 0)
                {
                    return false;
                }
            }

            return true;
        }

        /******************************************************************************
         * @brief Calculate 'count' number of primes.
         *
         * @param nCount - The number of primes to calculate.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void CalculatePrimes(int& nCount)
        {
            // Loop until we have the required amount of primes.
            if (int(m_vThreadPrimes.size()) < nCount)
            {
                // Check if our current number is a prime.
                if (IsPrime(m_nCurrentCount))
                {
                    m_vThreadPrimes.push_back(m_nCurrentCount);
                }

                // Increment counter.
                ++m_nCurrentCount;
            }
        }

        /******************************************************************************
         * @brief This code will run in a seperate thread. Main code goes here.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void ThreadedContinuousCode() override
        {
            // Change this to calculate a different number of prime numbers.
            CalculatePrimes(m_nCount);

            // Check if we have reached the desired number of primes.
            if (int(this->GetPrimes().size()) >= this->GetDesiredPrimeAmount())
            {
                // Call thread stop.
                this->RequestStop();
            }
        }

        /******************************************************************************
         * @brief Any highly parallelizable code that can be used in the main thread
         *       goes here.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0725
         ******************************************************************************/
        void PooledLinearCode() override {}

    public:
        // Declare and define public methods and variables.
        PrimeCalculatorThread() = default;

        /******************************************************************************
         * @brief Mutator for the Prime Count private member
         *
         * @param nNum - The amount of primes to calculate.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        void SetPrimeCount(int nNum) { m_nCount = nNum; }

        /******************************************************************************
         * @brief Accessor for the Prime Counter private member.
         *
         * @return int - The amount of primes the calculator will produce.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        int GetDesiredPrimeAmount() { return m_nCount; }

        /******************************************************************************
         * @brief Accessor for the Primes private member.
         *
         * @return std::vector<int> - A vector of the resultant primes.
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0722
         ******************************************************************************/
        std::vector<int> GetPrimes() { return m_vThreadPrimes; }

        /******************************************************************************
         * @brief Clears the prime results vector.
         *
         *
         * @author ClayJay3 (claytonraycowen@gmail.com)
         * @date 2023-0725
         ******************************************************************************/
        void ClearPrimes() { m_vThreadPrimes.clear(); }
};

/******************************************************************************
 * @brief Main example method.
 *
 * @return int - Return status.
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-0722
 ******************************************************************************/
void RunExample()
{
    // Declare prime calculator threads.
    PrimeCalculatorThread ThreadedPrimeCalculator1;
    PrimeCalculatorThread ThreadedPrimeCalculator2;
    PrimeCalculatorThread ThreadedPrimeCalculator3;
    PrimeCalculatorThread ThreadedPrimeCalculator4;
    PrimeCalculatorThread ThreadedPrimeCalculator5;

    /******************************************************************************
     * Seperate threads.
     ******************************************************************************/
    // Calc 1.
    ThreadedPrimeCalculator1.SetPrimeCount(5);
    ThreadedPrimeCalculator1.Start();
    // Calc 2.
    ThreadedPrimeCalculator2.SetPrimeCount(50);
    ThreadedPrimeCalculator2.Start();
    // Calc 3.
    ThreadedPrimeCalculator3.SetPrimeCount(10000);
    ThreadedPrimeCalculator3.Start();
    // Calc 4.
    ThreadedPrimeCalculator4.SetPrimeCount(500);
    ThreadedPrimeCalculator4.Start();
    // Calc 5. Example of a thread that takes to long.
    ThreadedPrimeCalculator5.SetPrimeCount(9999999);
    ThreadedPrimeCalculator5.Start();

    // Wait for threads to finish.
    ThreadedPrimeCalculator1.Join();
    ThreadedPrimeCalculator2.Join();
    ThreadedPrimeCalculator3.Join();
    ThreadedPrimeCalculator4.Join();
    // This thread will take took long, stop it prematurely.
    ThreadedPrimeCalculator5.RequestStop();
    ThreadedPrimeCalculator5.Join();

    // Print length of calculated primes vectors.
    std::cout << "Creating seperate threads:" << std::endl;
    std::vector<int> vPrimes = ThreadedPrimeCalculator1.GetPrimes();
    std::cout << "Calculator1 Primes Length: " << vPrimes.size() << std::endl;
    vPrimes = ThreadedPrimeCalculator2.GetPrimes();
    std::cout << "Calculator2 Primes Length: " << vPrimes.size() << std::endl;
    vPrimes = ThreadedPrimeCalculator3.GetPrimes();
    std::cout << "Calculator3 Primes Length: " << vPrimes.size() << std::endl;
    vPrimes = ThreadedPrimeCalculator4.GetPrimes();
    std::cout << "Calculator4 Primes Length: " << vPrimes.size() << std::endl;
    std::cout << "\n\nThis thread was stopped prematurely before reaching 9999999." << std::endl;
    vPrimes = ThreadedPrimeCalculator5.GetPrimes();
    std::cout << "Calculator5 Primes Length: " << vPrimes.size() << std::endl;

    // Clear Calculator5 and restart the thread.
    // Don't join before program exits to demonstrate
    // graceful shutdowns when user doesn't do what they're supposed to.
    ThreadedPrimeCalculator5.ClearPrimes();
    ThreadedPrimeCalculator5.Start();
    // Print info to user.
    std::cout << "Calculator5 was restarted and then main program exited without joining. (graceful shutdown demo)" << std::endl;
    vPrimes = ThreadedPrimeCalculator5.GetPrimes();
    std::cout << "Calculator5 Primes Length: " << vPrimes.size() << std::endl;
}
