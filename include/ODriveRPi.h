i#pragma once

#include <sstream>
#include "../include/RPiSerial.h"
#include "ODriveEnums.h"

/** @file ODriveRPi.h
 *  @addtogroup ODriveRPi
 *  @brief Port of ODriveArduino to Raspberry Pi using Serial Library
 *
 *  @namespace ODriveRPi
 *  @{
 */
namespace ODriveRPi {
    /**
    * @class ODriveRPi::ODriveRPi ODriveRPi.h "ODriveRPi.h"
    * @brief Communicates with ODrive v3.x, S1, and Pro via USB Serial
    * @details
    *  Function prototypes courtesy of ODriveArduino
    *  <br>
    *  https://github.com/odriverobotics/ODriveArduino/tree/master
    *
    *  This port of the ODrive library removes all Arduino dependencies. However, ODriveRPi does rely on the Serial library.
    *  Serial has built-in read timers, thus a timeout for readLine is not necessary, if data is present during
    *  the active interval it will be read.
    *  <p><strong>Has not been designed for and tested with hardware UART, SPI, or CAN.</strong><br>
    *  In use with such protocols please refer to ODrive's ASCII protocol documentation.</p>
    */
    class ODriveRPi {
    public:
        /** @struct ODriveFeedback ODriveRPi.h "ODriveRPi.h"
        *  @brief Stores position and velocity values retrieved from ODrive
        *
        *  @var ODriveRPi::ODriveFeedback::pos
        *  <strong>[rad]</strong> - Current position of servo as estimated by ODrive controller
        *  @var ODriveRPi::ODriveFeedback::vel
        *  <strong>[rad/s]</strong> - Current velocity of servo as estimated by ODrive controller
        */
        struct ODriveFeedback{
            float pos;
            float vel;
        };
        /** @brief
        * Instantiate a new ODriveRPi object
        *
        * @param[in] serial    <strong>[RPiSerial::Serial]</strong> - USB Serial interface to be used to communicate with ODrive
        *
        */
        explicit ODriveRPi(RPiSerial::RPiSerial &serial);

        /**
         * @brief Clears the error status of the ODrive and restarts the brake resistor if it was disabled due to an error.
         */
        void clearErrors();

        /**
         * @brief Sends a new position setpoint.
         *
         * @param[in] position <strong>[rad]</strong> - Servo position
         * @param[in] motorNum <strong>[int]</strong> - Servo (axis) number: 0 = Axis 0, 1 = Axis 1
         */
        void setPosition(float position, int motorNum);

        /**
         * @brief Sends a new position setpoint with a velocity feedforward term.
         *
         * @param[in] position              <strong>[rad]</strong> - Servo position
         * @param[in] velocity_feedforward  <strong>[rad/s]</strong> - Servo position
         * @param[in] motorNum              <strong>[int]</strong> - Servo (axis) number: 0 = Axis 0, 1 = Axis 1
         */
        void setPosition(float position, float velocity_feedforward, int motorNum);

        /**
         * @brief Sends a new position setpoint with velocity and torque feedforward terms.
         *
         * @param[in] position              <strong>[rad]</strong> - Servo position
         * @param[in] velocity_feedforward  <strong>[rad/s]</strong> - Servo velocity
         * @param[in] torque_feedforward    <strong>[rad/s^2]</strong> - Servo torque
         * @param[in] motorNum              <strong>[int]</strong> - Servo (axis) number: 0 = Axis 0, 1 = Axis 1
         */
        void setPosition(float position, float velocity_feedforward, float torque_feedforward, int motorNum);

        /**
         * @brief Requests the latest position and velocity estimates.
         * @param[in] motorNum <strong>[int]</strong> - Servo (axis) number: 0 = Axis 0, 1 = Axis 1
         *
         * @return feedback [ODriveFeedback] - Feedback retrieved from ODrive controller. Returns {NaN, NaN} in case of a communication error.
         */
        ODriveFeedback getFeedback(int motorNum);

        /**
         * @brief Requests the latest position estimate.
         *
         * @return pos <strong>[rad]</strong> Position retrieved from ODrive controller. Returns NaN in case of a communication error.
         */
        inline float getPosition(int motorNum) { return getFeedback(motorNum).pos; }

        /**
         * @brief Requests the latest position estimate.
         *
         * @return velocity <strong>[rad/s]</strong> Velocity retrieved from ODrive controller. Returns NaN in case of a communication error.
         */
        inline float getVelocity(int motorNum) { return getFeedback(motorNum).vel; }

        /**
         * @brief Requests a parameter from ODrive. To view available API endpoints refer to the official ODrive documentation
         * @param[in] path <strong>[std::string]</strong> - The query parameter to be sent to ODrive
         * @return param <strong>[std::string]</strong> - The value returned from the ODrive
         */
        std::string getParameterAsString(const std::string& path);

        /**
        * @brief Requests a parameter from ODrive. To view available API endpoints refer to the official ODrive documentation
        * @param[in] path <strong>[std::string]</strong> - The query parameter to be sent to ODrive
        * @return param <strong>[int]</strong> - The value returned from the ODrive
        */
        long getParameterAsInt(const std::string& path);
        /**
        * @brief Requests a parameter from ODrive. To view available API endpoints refer to the official ODrive documentation
        * @param[in] path <strong>[std::string]</strong> - The query parameter to be sent to ODrive
        * @return param <strong>[float]</strong> - The value returned from the ODrive
        */
        float getParameterAsFloat(const std::string& path);

        /**
        * @brief Requests to set a parameter from ODrive. To view the available parameters refer to the official ODrive documentation
        * @param[in] path <strong>[std::string]</strong> - The desired parameter be updated on ODrive
        * @param[in] value <strong>[std::string]</strong> - The value of the desired parameter
        */
        void setParameter(const std::string& path, const std::string& value);

        /**
        * @brief Requests to set a parameter from ODrive. To view the available parameters refer to the official ODrive documentation
        * @param[in] path <strong>[std::string]</strong> - The desired parameter be updated on ODrive
        * @param[in] value <strong>[long]</strong> - The value of the desired parameter
        */
        void setParameter(const std::string& path, long value) { setParameter(path, std::to_string(value)); }

        /**
         * @brief Tells the ODrive to change the state of the requested axis.
         *
         * @param[in] requested_state   <strong>[ODriveAxisState]</strong> - Desired ODrive axis state
         * @param[in] motorNum          <strong>[int]</strong> - Servo (axis) number: 0 = Axis 0, 1 = Axis 1
         */
        void setState(ODriveAxisState requested_state, int motorNum);

        /**
         * @brief Requests the current axis state from the ODrive.
         *
         * @return state [ODriveAxisState] - Returns the state of the requested axis. AXIS_STATE_UNDEFINED is returned in case of a communication error.
         */
        ODriveAxisState getState(int motorNum);

    private:
        RPiSerial::RPiSerial & _serial;
        std::stringstream _sstream;
        std::string readLine();
    };
}

/** @} */
