/*
   RoveCommPacket.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/23/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      
*/

#include <string>
#include "NetworkAddress.h"

#ifndef ROVECOMMPACKET_H
#define ROVECOMMPACKET_H

enum DataTypes {
    INT8_T,
    UINT8_T,
    INT16_T,
    UINT16_T,
    INT32_T,
    UINT32_T,
    FLOAT_T,
    DOUBLE_T,
    CHAR
};

template <typename T>
class RoveCommPacket {
private:
    int m_iDataId{};
    DataTypes m_eDataType;
    T m_tData;
    int m_iDataCount{};
    NetworkAddress m_sIPAddress;

public:
    RoveCommPacket();

    T GetDataPacket();

    void SetIPAddress(const std::string& szIPAddress, int iPort);
    std::string GetIPAddress();
    short unsigned int GetPort();
    char GetCData(DataTypes eType);
    void Print();

};

template<typename T>
T RoveCommPacket<T>::GetDataPacket() {
    return m_tData;
}

#include "RoveCommPacket.hpp"

#endif // ROVECOMMPACKET_H
