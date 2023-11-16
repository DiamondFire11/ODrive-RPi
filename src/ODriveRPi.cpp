// Port of ODriveArduino to Raspberry Pi using LCAPA Serial Library
// Built for use with ODrive v3.6, S1, and Pro

// ODriveArduino
// https://github.com/odriverobotics/ODriveArduino/tree/master

#include <iostream>
#include <cmath>
#include "../include/ODriveRPi.h"

ODriveRPi::ODriveRPi::ODriveRPi(RPiSerial::RPiSerial &serial) : _serial(serial){
    this->_sstream.str(""); // Initialize _sstream String Stream
}

void ODriveRPi::ODriveRPi::clearErrors() {
    this->_serial.send("sc\n");
}

void ODriveRPi::ODriveRPi::setPosition(float position, int motorNum) {
    setPosition(position, 0.0f, 0.0f, motorNum);
}

void ODriveRPi::ODriveRPi::setPosition(float position, float velocity_feedforward, int motorNum) {
    setPosition(position, velocity_feedforward, 0.0f, motorNum);
}

void ODriveRPi::ODriveRPi::setPosition(float position, float velocity_feedforward, float torque_feedforward, int motorNum) {
    this->_sstream.str(""); // Clear _sstream String Stream
    this->_sstream << "p " << motorNum << " " << position << " " << velocity_feedforward << " " << torque_feedforward << "\n";
    this->_serial.send(this->_sstream.str());
}

std::string ODriveRPi::ODriveRPi::getParameterAsString(const std::string &path) {
    this->_sstream.str(""); // Clear _sstream String Stream
    this->_sstream << "r " << path << "\n";
    this->_serial.send(this->_sstream.str());

    return this->readLine(); // Get response from ODrive
}

void ODriveRPi::ODriveRPi::setParameter(const std::string &path, const std::string &value) {
    this->_sstream.str(""); // Clear _sstream String Stream
    this->_sstream << "w " << path << " " << value << "\n";
    this->_serial.send(this->_sstream.str());
}

void ODriveRPi::ODriveRPi::setState(ODriveAxisState requested_state, int motorNum) {
    this->_sstream.str(""); // Clear _sstream String Stream
    this->_sstream << "axis" << motorNum << ".requested_state";
    setParameter(this->_sstream.str(), std::to_string((long) requested_state));
}

ODriveAxisState ODriveRPi::ODriveRPi::getState(int motorNum) {
    this->_sstream.str(""); // Clear _sstream String Stream
    this->_sstream << "axis" << motorNum << ".current_state";
    return ODriveAxisState(getParameterAsInt(this->_sstream.str()));
}

long ODriveRPi::ODriveRPi::getParameterAsInt(const std::string &path) {
    return std::stoi(getParameterAsString(path));
}

float ODriveRPi::ODriveRPi::getParameterAsFloat(const std::string &path) {
    return std::stof(getParameterAsString(path));
}

// Implement last
ODriveRPi::ODriveRPi::ODriveFeedback ODriveRPi::ODriveRPi::getFeedback(int motorNum) {
    unsigned char readBuf[256];
    this->_serial.recv(readBuf, sizeof(readBuf)); // Flush Serial FD

    this->_sstream.str(""); // Clear _sstream String Stream
    this->_sstream << "f " << motorNum << "\n";
    this->_serial.send(this->_sstream.str());

    std::string response = this->readLine();

    int spacePos = response.find(' ');
    if (spacePos >= 0) {
        return {
                std::stof(response.substr(0, spacePos)),
                std::stof(response.substr(spacePos+1))
        };
    } else {
        return {NAN, NAN};
    }
}

std::string ODriveRPi::ODriveRPi::readLine() {
    std::string response = "";
    unsigned char readBuf[256];

    do{
        // Read serial buffer
        long numRead = this->_serial.recv(readBuf, sizeof(readBuf)); // Flush Serial FD

        // Append values to string
        for (long i = 0; i < numRead; ++i) {
            response += char(readBuf[i]);
        }
    }while(response.find('\n') == std::string::npos);

    long newLinePos = response.find('\n');
    return response.substr(0, newLinePos);
}
