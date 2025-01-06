#include <Arduino.h>
#include <CrsfSerial.h>

class RX {
public:
  RX(HardwareSerial &port, uint32_t baud);
  void init();
  

private:
  CrsfSerial _crsf;

  void crsfLinkUp();
  void crsfLinkDown();
  void crsfPacket();
  void crsfOobData(uint8_t data);
  void crsfLinkStats();
  void crsfGPSPacket();
};