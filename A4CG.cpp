#include "Arduino.h"
#include "A4CG.h"

A4CG::A4CG(Stream& stream)
{
  this->_stream = &stream;
}

// Standby mode. For low power consumption and prolong the life of the sensor.
void A4CG::sleep()
{
  uint8_t command[] = { 0x33, 0x3E, 0x00, 0x0C, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1E };
  _stream->write(command, sizeof(command));
}

// Operating mode. Stable data should be got at least 30 seconds after the sensor wakeup from the sleep mode because of the fan's performance.
void A4CG::wakeUp()
{
  uint8_t command[] = { 0x33, 0x3E, 0x00, 0x0C, 0xA1, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1F };
  _stream->write(command, sizeof(command));
}

// Active mode. Default mode after power up. In this mode sensor would send serial data to the host automatically.
void A4CG::activeMode()
{
  uint8_t command[] = { 0x33, 0x3E, 0x00, 0x0C, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1F };
  _stream->write(command, sizeof(command));
  _mode = MODE_ACTIVE;
}

// Passive mode. In this mode sensor would send serial data to the host only for request.
void A4CG::passiveMode()
{
  uint8_t command[] = { 0x33, 0x3E, 0x00, 0x0C, 0xA2, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x20 };
  _stream->write(command, sizeof(command));
  _mode = MODE_PASSIVE;
}

// Request read in Passive Mode.
void A4CG::requestRead()
{
  if (_mode == MODE_PASSIVE)
  {
    uint8_t command[] = { 0x33, 0x3E, 0x00, 0x0C, 0xA4, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x21 };
    _stream->write(command, sizeof(command));
  }
}

// Non-blocking function for parse response.
bool A4CG::read(DATA& data)
{
  _data = &data;
  loop();
  
  return _status == STATUS_OK;
}

// Blocking function for parse response. Default timeout is 1s.
bool A4CG::readUntil(DATA& data, uint16_t timeout)
{
  _data = &data;
  uint32_t start = millis();
  do
  {
    loop();
    if (_status == STATUS_OK) break;
  } while (millis() - start < timeout);

  return _status == STATUS_OK;
}

void A4CG::loop()
{
  _status = STATUS_WAITING;
  if (_stream->available())
  {
    uint8_t ch = _stream->read();

    switch (_index)
    {
    case 0:
      if (ch != 0x32)
      {
        return;
      }
      _calculatedChecksum = ch;
      break;

    case 1:
      if (ch != 0x3D)
      {
        _index = 0;
        return;
      }
      _calculatedChecksum += ch;
      break;

    case 2:
      _calculatedChecksum += ch;
      _frameLen = ch << 8;
      break;

    case 3:
      _frameLen |= ch;
      // Unsupported sensor, different frame length, transmission error e.t.c.
      if (_frameLen != 2 * 5 + 2 && _frameLen != 2 * 13 + 2)
      {
        _index = 0;
        return;
      }
      _calculatedChecksum += ch;
      break;

// AllPass
    default:
      if (_index == _frameLen + 2)
      {
        _checksum = ch << 8;
      }
      else if (_index == _frameLen + 2 + 1)
      {
        _checksum |= ch;

        if (_calculatedChecksum == _checksum)
        {
          _status = STATUS_OK;

          // Standard Particles, CF=1.
          _data->PM_1_0 = makeWord(_payload[0], _payload[1]);
          _data->PM_2_5 = makeWord(_payload[2], _payload[3]);
          _data->PM_10_0 = makeWord(_payload[4], _payload[5]);

		//Number of particles
		_data->PM_NUM_0_3 = makeWord(_payload[6], _payload[7]);
		_data->PM_NUM_0_5 = makeWord(_payload[8], _payload[9]);
		_data->PM_NUM_1_0 = makeWord(_payload[10], _payload[11]);
		_data->PM_NUM_2_5 = makeWord(_payload[12], _payload[13]);
		_data->PM_NUM_5_0 = makeWord(_payload[14], _payload[15]);
		_data->PM_NUM_10_0 = makeWord(_payload[16], _payload[17]);
		}

        _index = 0;
        return;
      }
      else
      {
        _calculatedChecksum += ch;
        uint8_t payloadIndex = _index - 4;

        // Payload is common to all sensors (first 2x12 bytes).
        if (payloadIndex < sizeof(_payload))
        {
          _payload[payloadIndex] = ch;
        }
      }

      break;
    }

    _index++;
  }
}
