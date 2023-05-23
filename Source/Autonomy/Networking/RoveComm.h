/*
   RoveComm.h
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/21/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      
*/

#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "../Main/MRDT_Autonomy_Globals.h"

#ifndef ROVECOMM_H
#define ROVECOMM_H

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

enum NetworkAddressIntegers {
    NAI_OCTET_ONE,
    NAI_OCTET_TWO,
    NAI_OCTET_THREE,
    NAI_OCTET_FOUR,
    NAI_PORT
};

struct NetworkAddress {
    int m_iOctet1;
    int m_iOctet2;
    int m_iOctet3;
    int m_iOctet4;
    int m_iPort;

    NetworkAddress() {
        // IP Address
        this->m_iOctet1 = 0;
        this->m_iOctet2 = 0;
        this->m_iOctet3 = 0;
        this->m_iOctet4 = 0;

        // Port
        this->m_iPort = 0;
    }

    NetworkAddress(const std::string& szIPAddress, int iPort) {

        // Set the length of the IP Address String
        const unsigned long lIPAddressLength = szIPAddress.length();

        // Create a counter object for setting integer based IP Octets
        unsigned int iOctetCounter = 0;

        // Create Character Pointers for the substring process
        char* pIPAddress;
        char* pOctetOfIPAddress;

        // Create a new character array
        pIPAddress = new char[lIPAddressLength + 1];

        // Copy the passed in IP Address that was passed
        // into the function into the character array.
        strcpy(pIPAddress, szIPAddress.c_str());

        // Retrieve the first octet of the IP Address
        pOctetOfIPAddress = strtok(pIPAddress, ".");

        while (pOctetOfIPAddress != nullptr) {

            // Convert the octet character pointer into a string
            std::string szOctet = pOctetOfIPAddress;

            // Convert the string to an integer and assign it to the appropriate octet
            switch (iOctetCounter) {
                case 0  :  this->m_iOctet1 = std::stoi(szOctet)                        ;  break;
                case 1  :  this->m_iOctet2 = std::stoi(szOctet)                        ;  break;
                case 2  :  this->m_iOctet3 = std::stoi(szOctet)                        ;  break;
                case 3  :  this->m_iOctet4 = std::stoi(szOctet)                        ;  break;
                default :  PLOG_FATAL_(AL_ConsoleLogger) << "Reached Out of Bounds Index"  ;  break;
            }

            // Retrieve the next octet of the IP Address
            pOctetOfIPAddress = strtok(nullptr, "-");

            // Increment the Octet Counter
            iOctetCounter++;
        }

        // Set Port
        this->m_iPort = iPort;

        // Deinitialize Character Pointers
        delete pIPAddress;
        delete pOctetOfIPAddress;
    }

    int GetIData(NetworkAddressIntegers eKey) const {
        int iValue = 0;

        switch (eKey) {
            case NAI_OCTET_ONE    :  iValue = m_iOctet1                                         ;  break;
            case NAI_OCTET_TWO    :  iValue = m_iOctet2                                         ;  break;
            case NAI_OCTET_THREE  :  iValue = m_iOctet3                                         ;  break;
            case NAI_OCTET_FOUR   :  iValue = m_iOctet4                                         ;  break;
            case NAI_PORT         :  iValue = m_iPort                                           ;  break;
            default               :  PLOG_FATAL_(AL_ConsoleLogger) << "Reached invalid Index!"  ;  break;
        }

        return iValue;
    }
};

template <typename T>
struct RoveCommPacket {
    int m_iDataId;
    DataTypes m_eDataType;
    T m_tData;
    int m_iDataCount;
    NetworkAddress m_sIPAddress;

    void SetIPAddress(std::string szIPAddress, int iPort) {
        //this->m_sIPAddress.m_szIPAddress = szIPAddress;
        this->m_sIPAddress.m_iPort = iPort;
    }

    std::string GetIPAddress() {

        return "";

        // return m_sIPAddress.m_szIPAddress + ":" + std::to_string(m_sIPAddress.m_iPort);
    }

    char GetCData(DataTypes eType) {
        
        char cValue;

        switch (eType) {
            case INT8_T     :    cValue = 'b'                                                   ;   break;
            case UINT8_T    :    cValue = 'B'                                                   ;   break;
            case INT16_T    :    cValue = 'h'                                                   ;   break;
            case UINT16_T   :    cValue = 'H'                                                   ;   break;
            case INT32_T    :    cValue = 'l'                                                   ;   break;
            case UINT32_T   :    cValue = 'L'                                                   ;   break;
            case FLOAT_T    :    cValue = 'f'                                                   ;   break;
            case DOUBLE_T   :    cValue = 'd'                                                   ;   break;
            case CHAR       :    cValue = 'c'                                                   ;   break;
            default         :    PLOG_FATAL_(AL_ConsoleLogger) << "Reached invalid DataType!"   ;   break;
        }

        return cValue;
    }

    void Print() {
        std::printf("----------\n");
        std::printf("ID:    %d\n", m_iDataId);
        std::printf("Type:  %c\n", GetCData(m_eDataType));
        std::printf("Count: %d\n", m_iDataCount);
        std::printf("IP:    %s\n", GetIPAddress().c_str());

        switch (m_eDataType) {
            case INT8_T     :    std::printf("Data:  %d\n", int8_t(m_tData));                break;
            case UINT8_T    :    std::printf("Data:  %d\n", uint8_t(m_tData));               break;
            case INT16_T    :    std::printf("Data:  %d\n", int16_t(m_tData));               break;
            case UINT16_T   :    std::printf("Data:  %d\n", uint16_t(m_tData));              break;
            case INT32_T    :    std::printf("Data:  %d\n", int32_t(m_tData));               break;
            case UINT32_T   :    std::printf("Data:  %d\n", uint32_t(m_tData));              break;
            case FLOAT_T    :    std::printf("Data:  %f\n", float_t(m_tData));               break;
            case DOUBLE_T   :    std::printf("Data:  %f\n", double_t(m_tData));              PLOG_FATAL_(AL_ConsoleLogger) << "Reached invalid DataType!";      break;
            case CHAR       :    std::printf("Data:  %c\n", char(m_tData));                  break;
            default         :    PLOG_FATAL_(AL_ConsoleLogger) << "Reached invalid DataType!";      break;
        }

        std::printf("----------\n");
    }
};

class RoveComm {

};

#endif // ROVECOMM_H
