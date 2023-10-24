/******************************************************************************
 * @brief Defines and implements the TensorflowModel class.
 *
 * @file TensorflowModel.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-24
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef TENSORFLOW_MODEL_HPP
#define TENSORFLOW_MODEL_HPP

/******************************************************************************
 * @brief This class is designed to enable quick, easy, and robust handling of .tf and .tflite
 *      models. The class also makes it easy to automatically find an available device
 *      or a device can be specified.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-24
 ******************************************************************************/
class TensorflowModel
{
        /////////////////////////////////////////
        // Declare public enums that are specific to and used withing this class.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Construct a new Tensorflow Model object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-10-24
         ******************************************************************************/
        TensorflowModel()
        {
            // Initialize member variables.
        }

        /******************************************************************************
         * @brief Destroy the Tensorflow Model object.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2023-10-24
         ******************************************************************************/
        ~TensorflowModel()
        {
            // Nothing to destroy.
        }

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////

    private:
        /////////////////////////////////////////
        // Declare public methods.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
};

#endif
