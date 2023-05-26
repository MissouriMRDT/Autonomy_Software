/*
   RoveCommPacket.hpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/23/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:      
*/

template<typename T>
RoveCommPacket<T>::RoveCommPacket() {
    this->m_iDataId = 22000;
    this->m_eDataType = DOUBLE_T;
    this->m_tData = 124.24;
    this->m_iDataCount = 1;
    this->m_sIPAddress = {192, 168, 218, 1, 194};
}

template <typename T>
void RoveCommPacket<T>::SetIPAddress(const std::string& szIPAddress, int iPort) {
    //this->m_sIPAddress.m_szIPAddress = szIPAddress;
    this->m_sIPAddress.m_iPort = iPort;
}

template <typename T>
std::string RoveCommPacket<T>::GetIPAddress() {

    return m_sIPAddress.GetSZData();
}

template <typename T>
short unsigned int RoveCommPacket<T>::GetPort() {

    return m_sIPAddress.GetIData(NetworkAddressIntegers::NAI_PORT);
}



template <typename T>
char RoveCommPacket<T>::GetCData(DataTypes eType) {

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

template <typename T>
void RoveCommPacket<T>::Print() {
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
