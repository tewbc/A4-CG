#ifndef A4CG_H
#define A4CG_H

#include "Stream.h"

class A4CG
{
public:
  static const uint16_t SINGLE_RESPONSE_TIME = 1000;
  static const uint16_t TOTAL_RESPONSE_TIME = 1000 * 10;
  static const uint16_t STEADY_RESPONSE_TIME = 1000 * 30;

  static const uint16_t BAUD_RATE = 9600;

  struct DATA {
    // Standard Particles, CF=1
    uint16_t PM_1_0;
    uint16_t PM_2_5;
    uint16_t PM_10_0;

    // Number of particle
    uint16_t PM_NUM_0_3;
    uint16_t PM_NUM_0_5;
    uint16_t PM_NUM_1_0;
    uint16_t PM_NUM_2_5;
    uint16_t PM_NUM_5_0;
    uint16_t PM_NUM_10_0;
  };

  A4CG(Stream&);
  void sleep();
  void wakeUp();
  void activeMode();
  void passiveMode();

  void requestRead();
  bool read(DATA& data);
  bool readUntil(DATA& data, uint16_t timeout = SINGLE_RESPONSE_TIME);

private:
  enum STATUS { STATUS_WAITING, STATUS_OK };
  enum MODE { MODE_ACTIVE, MODE_PASSIVE };

  uint8_t _payload[18];				// For data 18 bytes as in datasheet.
  Stream* _stream;
  DATA* _data;
  STATUS _status;
  MODE _mode = MODE_ACTIVE;

  uint8_t _index = 0;
  uint16_t _frameLen;
  uint16_t _checksum;
  uint16_t _calculatedChecksum;

  void loop();
};

#endif
