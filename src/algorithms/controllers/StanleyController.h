/******************************************************************************
 * @brief Defines the StanleyController class within the controllers namespace.
 *
 * @file StanleyController.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H

/// \cond
// Put implicit includes in here.

/// \endcond

/******************************************************************************
 * @brief This namespace stores classes, functions, and structs that are used to
 *      implement different controllers that implement advanced control systems
 *      used for accurate and precise robotic control.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-02-01
 ******************************************************************************/
namespace controllers
{
    /******************************************************************************
     * @brief Provides an implementation of a lightweight lateral StanleyController.
     *      This algorithm is used to precisely control a different drive robot to
     *      follow a given path.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-02-01
     ******************************************************************************/
    class StanleyContoller
    {
        private:
            /////////////////////////////////////////
            // Declare private member variables.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Declare private methods.
            /////////////////////////////////////////

        public:
            /////////////////////////////////////////
            // Declare public member variables.
            /////////////////////////////////////////
            StanleyContoller();
            ~StanleyContoller();

            /////////////////////////////////////////
            // Declare public primary methods.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Setters.
            /////////////////////////////////////////

            /////////////////////////////////////////
            // Getters.
            ////////////////////////////////////////
    };
}    // namespace controllers

#endif
