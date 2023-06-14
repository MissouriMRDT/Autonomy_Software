/*
   RoveCommEthernetUDP.hpp
   Copyright (c) 2023 Mars Rover Design Team. All rights reserved.

   Date:             5/23/2023
   Author:           Eli Byrd and Clayton Cowen
   Description:
*/

template<typename T> RoveCommEthernetUDP<T>::RoveCommEthernetUDP()
{
	this->m_pNetworkAddress = {"192.168.1.166", 3303};
}

template<typename T> RoveCommEthernetUDP<T>::RoveCommEthernetUDP(int iPort)
{
	this->m_pNetworkAddress = {"192.168.1.166", iPort};
}

template<typename T> int RoveCommEthernetUDP<T>::Subscribe(NetworkAddress pSubscribeToIP) { return 0; }

template<typename T> int RoveCommEthernetUDP<T>::Write(RoveCommPacket<int8_t> pPacket) { return 0; }

template<typename T> int RoveCommEthernetUDP<T>::Write(RoveCommPacket<uint8_t> pPacket) { return 0; }

template<typename T> int RoveCommEthernetUDP<T>::Write(RoveCommPacket<int16_t> pPacket) { return 0; }

template<typename T> int RoveCommEthernetUDP<T>::Write(RoveCommPacket<uint16_t> pPacket) { return 0; }

template<typename T> int RoveCommEthernetUDP<T>::Write(RoveCommPacket<int32_t> pPacket) { return 0; }

template<typename T> int RoveCommEthernetUDP<T>::Write(RoveCommPacket<uint32_t> pPacket) { return 0; }

template<typename T> int RoveCommEthernetUDP<T>::Write(RoveCommPacket<float_t> pPacket) { return 0; }

template<typename T> int RoveCommEthernetUDP<T>::Write(RoveCommPacket<double_t> pPacket)
{
	try
	{
		boost::asio::io_context io_context;

		boost::asio::ip::udp::socket socket {io_context};
		socket.open(boost::asio::ip::udp::v4());

		socket.send_to(
			boost::asio::buffer(boost::lexical_cast<std::string>(pPacket.GetDataPacket())),
			boost::asio::ip::udp::endpoint {boost::asio::ip::make_address(pPacket.GetIPAddress()), pPacket.GetPort()});

		printf("SEND PACKET!\n");
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << '\n';
		return 1;
	}

	return 0;
}

template<typename T> int RoveCommEthernetUDP<T>::Write(RoveCommPacket<char> pPacket) { return 0; }

template<typename T> RoveCommPacket<T> RoveCommEthernetUDP<T>::ReadPacket() { return RoveCommPacket<T>(); }

template<typename T> void RoveCommEthernetUDP<T>::CloseSocket() {}
