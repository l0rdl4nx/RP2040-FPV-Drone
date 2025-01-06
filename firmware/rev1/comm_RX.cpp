#include "comm_RX.h"

RX::RX(HardwareSerial &port, uint32_t baud) :
  _crsf(port, baud)
{}

void RX::init() 
{
  _crsf.onLinkUp = &RX::crsfLinkUp;
  _crsf.onLinkDown = &RX::crsfLinkDown;
  _crsf.onOobData = &RX::crsfOobData;
  _crsf.onPacketChannels = &RX::crsfPacket;
  _crsf.onPacketLinkStatistics = &RX::crsfLinkStats;
  _crsf.onPacketGPS = &RX::crsfGPSPacket;
  _crsf.begin();
}

void RX::crsfLinkUp() 
{

}

void RX::crsfLinkDown()
{

}

void RX::crsfPacket()
{

}

void RX::crsfOobData(uint8_t data) 
{

}

void RX:crsfLinkStats()
{

}

void RX::crsfGPSPacket()
{

}