/******************************************************************************
 * @brief Unit/Integration Test Suite Example
 *
 * @file Example.cc
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "./TestingBase.hh"

/// \cond
#include <gtest/gtest.h>

/// \endcond

/******************************************************************************
 * @brief Unit Test Class for the Example Tests
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-10
 ******************************************************************************/
class ExampleTests : public TestingBase<ExampleTests>
{
    private:
        // Please note that any functions or variables must be declared as protected or public
        // for the tests to be able to directly access them.

    protected:
        int m_nTestValue;

    public:
        /******************************************************************************
         * @brief Construct a new Example Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ExampleTests() {}

        /******************************************************************************
         * @brief Destroy the Example Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        ~ExampleTests() {}

        /******************************************************************************
         * @brief Setup the Example Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestSetup() override { m_nTestValue = 0; }

        /******************************************************************************
         * @brief Destroy the Example Tests object.
         *
         * @author Eli Byrd (edbgkk@mst.edu)
         * @date 2025-01-10
         ******************************************************************************/
        void TestTeardown() override { EXPECT_NE(m_nTestValue, 0) << "Test value has not been modified."; }
};

/******************************************************************************
 * @brief Test the addition of two numbers.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-10
 ******************************************************************************/
TEST_F(ExampleTests, Addition)
{
    int a = 2, b = 3;
    m_nTestValue = a + b;

    EXPECT_EQ(m_nTestValue, 5) << "Addition test failed.";
}

/******************************************************************************
 * @brief Test the subtraction of two numbers.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-10
 ******************************************************************************/
TEST_F(ExampleTests, Subtraction)
{
    int a = 5, b = 3;
    m_nTestValue = a - b;

    EXPECT_EQ(m_nTestValue, 2) << "Subtraction test failed.";
}

/******************************************************************************
 * @brief Test the multiplication of two numbers.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-10
 ******************************************************************************/
TEST_F(ExampleTests, Multiplication)
{
    int a = 4, b = 5;
    m_nTestValue = a * b;

    EXPECT_EQ(m_nTestValue, 20) << "Multiplication test failed.";
}

/******************************************************************************
 * @brief Test the division of two numbers.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2025-01-10
 ******************************************************************************/
TEST_F(ExampleTests, Division)
{
    int a = 10, b = 2;
    m_nTestValue = a / b;

    EXPECT_EQ(m_nTestValue, 5) << "Division test failed.";
}
