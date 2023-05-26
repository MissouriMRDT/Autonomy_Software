/*
   RoveCommEthernetUDP.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/23/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      
*/

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include "NetworkAddress.h"
#include "RoveCommPacket.h"

#ifndef ROVECOMMETHERNETUDP_H
#define ROVECOMMETHERNETUDP_H

template <typename T>
class RoveCommEthernetUDP {
private:
    NetworkAddress m_pNetworkAddress;
    // TODO: Subscribers

public:
    RoveCommEthernetUDP();
    RoveCommEthernetUDP(int iPort);

    int Subscribe(NetworkAddress pSubscribeToIP);

    int Write(RoveCommPacket<int8_t> pPacket);
    int Write(RoveCommPacket<uint8_t> pPacket);
    int Write(RoveCommPacket<int16_t> pPacket);
    int Write(RoveCommPacket<uint16_t> pPacket);
    int Write(RoveCommPacket<int32_t> pPacket);
    int Write(RoveCommPacket<uint32_t> pPacket);
    int Write(RoveCommPacket<float_t> pPacket);
    int Write(RoveCommPacket<double_t> pPacket);
    int Write(RoveCommPacket<char> pPacket);

    RoveCommPacket<T> ReadPacket();

    void CloseSocket();
};

#include "RoveCommEthernetUDP.hpp"

#endif // ROVECOMMETHERNETUDP_H
