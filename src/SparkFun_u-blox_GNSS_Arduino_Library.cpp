/*
  This is a library written for the u-blox ZED-F9P and NEO-M8P-2
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/16481
  https://www.sparkfun.com/products/15136
  https://www.sparkfun.com/products/15005
  https://www.sparkfun.com/products/15733
  https://www.sparkfun.com/products/15193
  https://www.sparkfun.com/products/15210

  Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
  v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020

  This library handles configuring and handling the responses
  from a u-blox GPS module. Works with most modules from u-blox including
  the Zed-F9P, NEO-M8P-2, NEO-M9N, ZOE-M8Q, SAM-M8Q, and many others.

  https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.13

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  The MIT License (MIT)
  Copyright (c) 2016 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

SFE_UBLOX_GNSS::SFE_UBLOX_GNSS(void)
{
  // Constructor
  if (debugPin >= 0)
  {
    pinMode((uint8_t)debugPin, OUTPUT);
    digitalWrite((uint8_t)debugPin, HIGH);
  }

  _logNMEA.all = 0;                             // Default to passing no NMEA messages to the file buffer
  _processNMEA.all = SFE_UBLOX_FILTER_NMEA_ALL; // Default to passing all NMEA messages to processNMEA

  // Support for platforms like ESP32 which do not support multiple I2C restarts
  // If _i2cStopRestart is true, endTransmission will always use a stop. If false, a restart will be used where needed.
#if defined(ARDUINO_ARCH_ESP32)
  _i2cStopRestart = true; // Always use a stop
#else
  _i2cStopRestart = false; // Use a restart where needed
#endif
}

SFE_UBLOX_GNSS::~SFE_UBLOX_GNSS(void)
{
  // Destructor

  end(); // Delete all allocated memory - excluding payloadCfg, payloadAuto and spiBuffer

  if (payloadCfg != NULL)
  {
    delete[] payloadCfg; // Created with new[]
    payloadCfg = NULL;   // Redundant?
  }

  if (payloadAuto != NULL)
  {
    delete[] payloadAuto; // Created with new[]
    payloadAuto = NULL;   // Redundant?
  }
}

// Stop all automatic message processing. Free all used RAM
void SFE_UBLOX_GNSS::end(void)
{
  // Note: payloadCfg is not deleted

  // Note: payloadAuto is not deleted

  // Note: spiBuffer is not deleted

  if (ubxFileBuffer != NULL) // Check if RAM has been allocated for the file buffer
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("end: the file buffer has been deleted. You will need to call setFileBufferSize before .begin to create a new one."));
    }
#endif
    delete[] ubxFileBuffer; // Created with new[]
    ubxFileBuffer = NULL;   // Redundant?
    fileBufferSize = 0;     // Reset file buffer size. User will have to call setFileBufferSize again
    fileBufferMaxAvail = 0;
  }

  if (moduleSWVersion != NULL)
  {
    delete moduleSWVersion; // Created with new moduleSWVersion_t
    moduleSWVersion = NULL; // Redundant?
  }

  if (packetUBXNAVPVT != NULL)
  {
    if (packetUBXNAVPVT->callbackData != NULL)
    {
      delete packetUBXNAVPVT->callbackData;
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("end: packetUBXNAVPVT->callbackData has been deleted"));
      }
#endif
    }
    delete packetUBXNAVPVT;
    packetUBXNAVPVT = NULL; // Redundant?
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("end: packetUBXNAVPVT has been deleted"));
    }
#endif
  }
}

// Allow the user to change packetCfgPayloadSize. Handy if you want to process big messages like RAWX
// This can be called before .begin if required / desired
bool SFE_UBLOX_GNSS::setPacketCfgPayloadSize(size_t payloadSize)
{
  bool success = true;

  if ((payloadSize == 0) && (payloadCfg != NULL))
  {
    // Zero payloadSize? Dangerous! But we'll free the memory anyway...
    delete[] payloadCfg; // Created with new[]
    payloadCfg = NULL;   // Redundant?
    packetCfg.payload = payloadCfg;
    packetCfgPayloadSize = payloadSize;
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setPacketCfgPayloadSize: Zero payloadSize!"));
  }

  else if (payloadCfg == NULL) // Memory has not yet been allocated - so use new
  {
    payloadCfg = new uint8_t[payloadSize];
    packetCfg.payload = payloadCfg;
    if (payloadCfg == NULL)
    {
      success = false;
      packetCfgPayloadSize = 0;
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial->println(F("setPacketCfgPayloadSize: RAM alloc failed!"));
    }
    else
      packetCfgPayloadSize = payloadSize;
  }

  else // Memory has already been allocated - so resize
  {
    uint8_t *newPayload = new uint8_t[payloadSize];

    if (newPayload == NULL) // Check if the alloc was successful
    {
      success = false;                                           // Report failure. Don't change payloadCfg, packetCfg.payload or packetCfgPayloadSize
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial->println(F("setPacketCfgPayloadSize: RAM resize failed!"));
    }
    else
    {
      memcpy(newPayload, payloadCfg, payloadSize <= packetCfgPayloadSize ? payloadSize : packetCfgPayloadSize); // Copy as much existing data as we can
      delete[] payloadCfg;                                                                                      // Free payloadCfg. Created with new[]
      payloadCfg = newPayload;                                                                                  // Point to the newPayload
      packetCfg.payload = payloadCfg;                                                                           // Update the packet pointer
      packetCfgPayloadSize = payloadSize;                                                                       // Update the packet payload size
    }
  }

  return (success);
}

// Return the number of free bytes remaining in packetCfgPayload
size_t SFE_UBLOX_GNSS::getPacketCfgSpaceRemaining()
{
  return (packetCfgPayloadSize - packetCfg.len);
}

// Initialize the I2C port
bool SFE_UBLOX_GNSS::begin(TwoWire &wirePort, uint8_t deviceAddress, uint16_t maxWait, bool assumeSuccess)
{
  commType = COMM_TYPE_I2C;
  _i2cPort = &wirePort; // Grab which port the user wants us to use
  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // We expect caller to begin their I2C port, with the speed of their choice external to the library
  // But if they forget, we start the hardware here.

  // We're moving away from the practice of starting Wire hardware in a library. This is to avoid cross platform issues.
  // ie, there are some platforms that don't handle multiple starts to the wire hardware. Also, every time you start the wire
  // hardware the clock speed reverts back to 100kHz regardless of previous Wire.setClocks().
  //_i2cPort->begin();

  _gpsI2Caddress = deviceAddress; // Store the I2C address from user

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  // New in v2.0: allocate memory for the file buffer - if required. (The user should have called setFileBufferSize already)
  createFileBuffer();

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - second attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - third attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: third attempt failed. Assuming success..."));
    }
#endif
    return (true);
  }

  return (connected);
}

// Allow the user to change I2C polling wait (the minimum interval between I2C data requests - to avoid pounding the bus)
// i2cPollingWait defaults to 100ms and is adjusted automatically when setNavigationFrequency()
// or setHNRNavigationRate() are called. But if the user is using callbacks, it might be advantageous
// to be able to set the polling wait manually.
void SFE_UBLOX_GNSS::setI2CpollingWait(uint8_t newPollingWait_ms)
{
  i2cPollingWait = newPollingWait_ms;
}

// Sets the global size for I2C transactions
// Most platforms use 32 bytes (the default) but this allows users to increase the transaction
// size if the platform supports it
// Note: If the transaction size is set larger than the platforms buffer size, bad things will happen.
void SFE_UBLOX_GNSS::setI2CTransactionSize(uint8_t transactionSize)
{
  if (transactionSize < 8)
    transactionSize = 8; // Ensure transactionSize is at least 8 bytes otherwise sendI2cCommand will have problems!

  i2cTransactionSize = transactionSize;
}
uint8_t SFE_UBLOX_GNSS::getI2CTransactionSize(void)
{
  return (i2cTransactionSize);
}

// Returns true if I2C device ack's
bool SFE_UBLOX_GNSS::isConnected(uint16_t maxWait)
{
  if (commType == COMM_TYPE_I2C)
  {
    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
    if (_i2cPort->endTransmission() != 0)
      return false; // Sensor did not ack
  }

  // Query port configuration to see whether we get a meaningful response
  // We could simply request the config for any port but, just for giggles, let's request the config for most appropriate port
  if (commType == COMM_TYPE_I2C)
    return (getPortSettingsInternal(COM_PORT_I2C, maxWait));
  else if (commType == COMM_TYPE_SERIAL)
    return (getPortSettingsInternal(COM_PORT_UART1, maxWait)); // Could be UART2 - but this is just a response check
  else                                                         // if (commType == COMM_TYPE_SPI)
    return (getPortSettingsInternal(COM_PORT_SPI, maxWait));
}

// Enable or disable the printing of sent/response HEX values.
// Use this in conjunction with 'Transport Logging' from the Universal Reader Assistant to see what they're doing that we're not
void SFE_UBLOX_GNSS::enableDebugging(Stream &debugPort, bool printLimitedDebug)
{
  _debugSerial = &debugPort; // Grab which port the user wants us to use for debugging
  if (printLimitedDebug == false)
  {
    _printDebug = true; // Should we print the commands we send? Good for debugging
  }
  else
  {
    _printLimitedDebug = true; // Should we print limited debug messages? Good for debugging high navigation rates
  }
}
void SFE_UBLOX_GNSS::disableDebugging(void)
{
  _printDebug = false; // Turn off extra print statements
  _printLimitedDebug = false;
}

// Safely print messages
void SFE_UBLOX_GNSS::debugPrint(char *message)
{
  if (_printDebug == true)
  {
    _debugSerial->print(message);
  }
}
// Safely print messages
void SFE_UBLOX_GNSS::debugPrintln(char *message)
{
  if (_printDebug == true)
  {
    _debugSerial->println(message);
  }
}

const char *SFE_UBLOX_GNSS::statusString(sfe_ublox_status_e stat)
{
  switch (stat)
  {
  case SFE_UBLOX_STATUS_SUCCESS:
    return "Success";
    break;
  case SFE_UBLOX_STATUS_FAIL:
    return "General Failure";
    break;
  case SFE_UBLOX_STATUS_CRC_FAIL:
    return "CRC Fail";
    break;
  case SFE_UBLOX_STATUS_TIMEOUT:
    return "Timeout";
    break;
  case SFE_UBLOX_STATUS_COMMAND_NACK:
    return "Command not acknowledged (NACK)";
    break;
  case SFE_UBLOX_STATUS_OUT_OF_RANGE:
    return "Out of range";
    break;
  case SFE_UBLOX_STATUS_INVALID_ARG:
    return "Invalid Arg";
    break;
  case SFE_UBLOX_STATUS_INVALID_OPERATION:
    return "Invalid operation";
    break;
  case SFE_UBLOX_STATUS_MEM_ERR:
    return "Memory Error";
    break;
  case SFE_UBLOX_STATUS_HW_ERR:
    return "Hardware Error";
    break;
  case SFE_UBLOX_STATUS_DATA_SENT:
    return "Data Sent";
    break;
  case SFE_UBLOX_STATUS_DATA_RECEIVED:
    return "Data Received";
    break;
  case SFE_UBLOX_STATUS_I2C_COMM_FAILURE:
    return "I2C Comm Failure";
    break;
  case SFE_UBLOX_STATUS_DATA_OVERWRITTEN:
    return "Data Packet Overwritten";
    break;
  default:
    return "Unknown Status";
    break;
  }
  return "None";
}

// Check for the arrival of new I2C/Serial/SPI data

// Called regularly to check for available bytes on the user' specified port
bool SFE_UBLOX_GNSS::checkUblox(uint8_t requestedClass, uint8_t requestedID)
{
  return checkUbloxInternal(&packetCfg, requestedClass, requestedID);
}

// PRIVATE: Called regularly to check for available bytes on the user' specified port
bool SFE_UBLOX_GNSS::checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (commType == COMM_TYPE_I2C)
    return (checkUbloxI2C(incomingUBX, requestedClass, requestedID));
  return false;
}

// Polls I2C for data, passing any new bytes to process()
// Returns true if new bytes are available
bool SFE_UBLOX_GNSS::checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (millis() - lastCheck >= i2cPollingWait)
  {
    // Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _i2cPort->beginTransmission(_gpsI2Caddress);
    _i2cPort->write(0xFD);                               // 0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    uint8_t i2cError = _i2cPort->endTransmission(false); // Always send a restart command. Do not release the bus. ESP32 supports this.
    if (i2cError != 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("checkUbloxI2C: I2C error: endTransmission returned "));
        _debugSerial->println(i2cError);
      }
#endif
      return (false); // Sensor did not ACK
    }

    // Forcing requestFrom to use a restart would be unwise. If bytesAvailable is zero, we want to surrender the bus.
    uint8_t bytesReturned = _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, static_cast<uint8_t>(2));
    if (bytesReturned != 2)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("checkUbloxI2C: I2C error: requestFrom 0xFD returned "));
        _debugSerial->println(bytesReturned);
      }
#endif
      return (false); // Sensor did not return 2 bytes
    }
    else // if (_i2cPort->available())
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      // if (lsb == 0xFF)
      // {
      //   //I believe this is a u-blox bug. Device should never present an 0xFF.
      //   if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      //   {
      //     _debugSerial->print(F("checkUbloxI2C: u-blox bug? Length lsb is 0xFF. i2cPollingWait is "));
      //     _debugSerial->println(i2cPollingWait);
      //   }
      //   if (debugPin >= 0)
      //   {
      //     digitalWrite((uint8_t)debugPin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)debugPin, HIGH);
      //   }
      //   lastCheck = millis(); //Put off checking to avoid I2C bus traffic
      //   return (false);
      // }
      // if (msb == 0xFF)
      // {
      //   //I believe this is a u-blox bug. Device should never present an 0xFF.
      //   if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      //   {
      //     _debugSerial->print(F("checkUbloxI2C: u-blox bug? Length msb is 0xFF. i2cPollingWait is "));
      //     _debugSerial->println(i2cPollingWait);
      //   }
      //   if (debugPin >= 0)
      //   {
      //     digitalWrite((uint8_t)debugPin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)debugPin, HIGH);
      //   }
      //   lastCheck = millis(); //Put off checking to avoid I2C bus traffic
      //   return (false);
      // }
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    if (bytesAvailable == 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("checkUbloxI2C: OK, zero bytes available"));
      }
#endif
      lastCheck = millis(); // Put off checking to avoid I2C bus traffic
      return (false);
    }

    // Check for undocumented bit error. We found this doing logic scans.
    // This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
    // then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
    if (bytesAvailable & ((uint16_t)1 << 15))
    {
      // Clear the MSbit
      bytesAvailable &= ~((uint16_t)1 << 15);

      // if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      // {
      //   _debugSerial->print(F("checkUbloxI2C: Bytes available error: "));
      //   _debugSerial->println(bytesAvailable);
      //   if (debugPin >= 0)
      //   {
      //     digitalWrite((uint8_t)debugPin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)debugPin, HIGH);
      //   }
      // }
    }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (bytesAvailable > 100)
    {
      if (_printDebug == true)
      {
        _debugSerial->print(F("checkUbloxI2C: Large packet of "));
        _debugSerial->print(bytesAvailable);
        _debugSerial->println(F(" bytes received"));
      }
    }
    else
    {
      if (_printDebug == true)
      {
        _debugSerial->print(F("checkUbloxI2C: Reading "));
        _debugSerial->print(bytesAvailable);
        _debugSerial->println(F(" bytes"));
      }
    }
#endif

    while (bytesAvailable)
    {
      // From the u-blox integration manual:
      // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
      //  address and thus allows any register to be read. The second "current address" form omits the
      //  register address. If this second form is used, then an address pointer in the receiver is used to
      //  determine which register to read. This address pointer will increment after each read unless it
      //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
      //  unaltered."
      // This means that after reading bytesAvailable from 0xFD and 0xFE, the address pointer will already be
      // pointing at 0xFF, so we do not need to write it here. The next four lines can be commented.
      //_i2cPort->beginTransmission(_gpsI2Caddress);
      //_i2cPort->write(0xFF);                     //0xFF is the register to read data from
      // if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
      //  return (false);                          //Sensor did not ACK

      // Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable; // 16-bit
      if (bytesToRead > i2cTransactionSize)  // Limit for i2cTransactionSize is 8-bit
        bytesToRead = i2cTransactionSize;

      // TRY_AGAIN:

      // Here it would be desireable to use a restart where possible / supported, but only if there will be multiple reads.
      // However, if an individual requestFrom fails, we could end up leaving the bus hanging.
      // On balance, it is probably safest to not use restarts here.
      uint8_t bytesReturned = _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)bytesToRead);
      if ((uint16_t)bytesReturned == bytesToRead)
      {
        for (uint16_t x = 0; x < bytesToRead; x++)
        {
          uint8_t incoming = _i2cPort->read(); // Grab the actual character

          // Check to see if the first read is 0x7F. If it is, the module is not ready to respond. Stop, wait, and try again.
          // Note: the integration manual says:
          //"If there is no data awaiting transmission from the receiver, then this register will deliver the value 0xFF,
          //  which cannot be the first byte of a valid message."
          // But it can be the first byte waiting to be read from the buffer if we have already read part of the message.
          // Therefore I think this check needs to be commented.
          //  if (x == 0)
          //  {
          //    if ((incoming == 0x7F) && (ubx7FcheckDisabled == false))
          //    {
          //      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          //      {
          //        _debugSerial->println(F("checkUbloxU2C: u-blox error, module not ready with data (7F error)"));
          //      }
          //      delay(5); //In logic analyzation, the module starting responding after 1.48ms
          //      if (debugPin >= 0)
          //      {
          //        digitalWrite((uint8_t)debugPin, LOW);
          //        delay(10);
          //        digitalWrite((uint8_t)debugPin, HIGH);
          //      }
          //      goto TRY_AGAIN;
          //    }
          //  }

          process(incoming, incomingUBX, requestedClass, requestedID); // Process this valid character
        }
      }
      else
        return (false); // Sensor did not respond

      bytesAvailable -= bytesToRead;
    }
  }

  return (true);

} // end checkUbloxI2C()

// PRIVATE: Check if we have storage allocated for an incoming "automatic" message
bool SFE_UBLOX_GNSS::checkAutomatic(uint8_t Class, uint8_t ID)
{
  bool result = false;
  switch (Class)
  {
  case UBX_CLASS_NAV:
  {
    switch (ID)
    {
    case UBX_NAV_PVT:
      if (packetUBXNAVPVT != NULL)
        result = true;
      break;
    }
  }
  break;
  }
  return (result);
}

// PRIVATE: Calculate how much RAM is needed to store the payload for a given automatic message
uint16_t SFE_UBLOX_GNSS::getMaxPayloadSize(uint8_t Class, uint8_t ID)
{
  uint16_t maxSize = 0;
  switch (Class)
  {
  case UBX_CLASS_NAV:
  {
    switch (ID)
    {
      case UBX_NAV_PVT:
        maxSize = UBX_NAV_PVT_LEN;
        break;
    }
  }
  }
  return (maxSize);
}

// Processes NMEA and UBX binary sentences one byte at a time
// Take a given byte and file it into the proper array
void SFE_UBLOX_GNSS::process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (_outputPort != NULL)
    _outputPort->write(incoming); // Echo this byte to the serial port
  if ((currentSentence == SFE_UBLOX_SENTENCE_TYPE_NONE) || (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA))
  {
    if (incoming == UBX_SYNCH_1) // UBX binary frames start with 0xB5, aka μ
    {
      // This is the start of a binary sentence. Reset flags.
      // We still don't know the response class
      ubxFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_UBX;
      // Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
      packetBuf.counter = 0;
      ignoreThisPayload = false; // We should not ignore this payload - yet
      // Store data in packetBuf until we know if we have a requested class and ID match
      activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
    }
    else if (incoming == '$')
    {
      nmeaByteCounter = 0; // Reset the NMEA byte counter
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NMEA;
    }
    else if (incoming == 0xD3) // RTCM frames start with 0xD3
    {
      rtcmFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_RTCM;
    }
    else
    {
      // This character is unknown or we missed the previous start of a sentence
    }
  }

  // Depending on the sentence, pass the character to the individual processor
  if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_UBX)
  {
    // Decide what type of response this is
    if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1))      // ISO 'μ'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2)) // ASCII 'b'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    // Note to future self:
    // There may be some duplication / redundancy in the next few lines as processUBX will also
    // load information into packetBuf, but we'll do it here too for clarity
    else if (ubxFrameCounter == 2) // Class
    {
      // Record the class in packetBuf until we know what to do with it
      packetBuf.cls = incoming; // (Duplication)
      rollingChecksumA = 0;     // Reset our rolling checksums here (not when we receive the 0xB5)
      rollingChecksumB = 0;
      packetBuf.counter = 0;                                   // Reset the packetBuf.counter (again)
      packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
      packetBuf.startingSpot = incomingUBX->startingSpot;      // Copy the startingSpot
    }
    else if (ubxFrameCounter == 3) // ID
    {
      // Record the ID in packetBuf until we know what to do with it
      packetBuf.id = incoming; // (Duplication)
      // We can now identify the type of response
      // If the packet we are receiving is not an ACK then check for a class and ID match
      if (packetBuf.cls != UBX_CLASS_ACK)
      {
        // This is not an ACK so check for a class and ID match
        if ((packetBuf.cls == requestedClass) && (packetBuf.id == requestedID))
        {
          // This is not an ACK and we have a class and ID match
          // So start diverting data into incomingUBX (usually packetCfg)
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
        }
        // This is not an ACK and we do not have a complete class and ID match
        // So let's check if this is an "automatic" message which has its own storage defined
        else if (checkAutomatic(packetBuf.cls, packetBuf.id))
        {
          // This is not the message we were expecting but it has its own storage and so we should process it anyway.
          // We'll try to use packetAuto to buffer the message (so it can't overwrite anything in packetCfg).
          // We need to allocate memory for the packetAuto payload (payloadAuto) - and delete it once
          // reception is complete.
          uint16_t maxPayload = getMaxPayloadSize(packetBuf.cls, packetBuf.id); // Calculate how much RAM we need
          if (maxPayload == 0)
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->print(F("process: getMaxPayloadSize returned ZERO!! Class: 0x"));
              _debugSerial->print(packetBuf.cls);
              _debugSerial->print(F(" ID: 0x"));
              _debugSerial->println(packetBuf.id);
            }
#endif
          }
          if (payloadAuto != NULL) // Check if memory is already allocated - this should be impossible!
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->println(F("process: memory is already allocated for payloadAuto! Deleting..."));
            }
#endif
            delete[] payloadAuto; // Created with new[]
            payloadAuto = NULL;   // Redundant?
            packetAuto.payload = payloadAuto;
          }
          payloadAuto = new uint8_t[maxPayload]; // Allocate RAM for payloadAuto
          packetAuto.payload = payloadAuto;
          if (payloadAuto == NULL) // Check if the alloc failed
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->print(F("process: memory allocation failed for \"automatic\" message: Class: 0x"));
              _debugSerial->print(packetBuf.cls, HEX);
              _debugSerial->print(F(" ID: 0x"));
              _debugSerial->println(packetBuf.id, HEX);
              _debugSerial->println(F("process: \"automatic\" message could overwrite data"));
            }
#endif
            // The RAM allocation failed so fall back to using incomingUBX (usually packetCfg) even though we risk overwriting data
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
            incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
            incomingUBX->id = packetBuf.id;
            incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
          }
          else
          {
            // The RAM allocation was successful so we start diverting data into packetAuto and process it
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETAUTO;
            packetAuto.cls = packetBuf.cls; // Copy the class and ID into packetAuto
            packetAuto.id = packetBuf.id;
            packetAuto.counter = packetBuf.counter;           // Copy over the .counter too
            packetAuto.startingSpot = packetBuf.startingSpot; // And the starting spot? (Probably redundant)
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if (_printDebug == true)
            {
              _debugSerial->print(F("process: incoming \"automatic\" message: Class: 0x"));
              _debugSerial->print(packetBuf.cls, HEX);
              _debugSerial->print(F(" ID: 0x"));
              _debugSerial->println(packetBuf.id, HEX);
            }
#endif
          }
        }
        else
        {
          // This is not an ACK and we do not have a class and ID match
          // so we should keep diverting data into packetBuf and ignore the payload
          ignoreThisPayload = true;
        }
      }
      else
      {
        // This is an ACK so it is to early to do anything with it
        // We need to wait until we have received the length and data bytes
        // So we should keep diverting data into packetBuf
      }
    }
    else if (ubxFrameCounter == 4) // Length LSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len = incoming; // (Duplication)
    }
    else if (ubxFrameCounter == 5) // Length MSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len |= incoming << 8; // (Duplication)
    }
    else if (ubxFrameCounter == 6) // This should be the first byte of the payload unless .len is zero
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial->print(F("process: ZERO LENGTH packet received: Class: 0x"));
          _debugSerial->print(packetBuf.cls, HEX);
          _debugSerial->print(F(" ID: 0x"));
          _debugSerial->println(packetBuf.id, HEX);
        }
#endif
        // If length is zero (!) this will be the first byte of the checksum so record it
        packetBuf.checksumA = incoming;
      }
      else
      {
        // The length is not zero so record this byte in the payload
        packetBuf.payload[0] = incoming;
      }
    }
    else if (ubxFrameCounter == 7) // This should be the second byte of the payload unless .len is zero or one
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
        // If length is zero (!) this will be the second byte of the checksum so record it
        packetBuf.checksumB = incoming;
      }
      else if (packetBuf.len == 1) // Check if length is one
      {
        // The length is one so this is the first byte of the checksum
        packetBuf.checksumA = incoming;
      }
      else // Length is >= 2 so this must be a payload byte
      {
        packetBuf.payload[1] = incoming;
      }
      // Now that we have received two payload bytes, we can check for a matching ACK/NACK
      if ((activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
          && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
          && (packetBuf.payload[0] == requestedClass)        // and if the class matches
          && (packetBuf.payload[1] == requestedID))          // and if the ID matches
      {
        if (packetBuf.len == 2) // Check if .len is 2
        {
          // Then this is a matching ACK so copy it into packetAck
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETACK;
          packetAck.cls = packetBuf.cls;
          packetAck.id = packetBuf.id;
          packetAck.len = packetBuf.len;
          packetAck.counter = packetBuf.counter;
          packetAck.payload[0] = packetBuf.payload[0];
          packetAck.payload[1] = packetBuf.payload[1];
        }
        else // Length is not 2 (hopefully this is impossible!)
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial->print(F("process: ACK received with .len != 2: Class: 0x"));
            _debugSerial->print(packetBuf.payload[0], HEX);
            _debugSerial->print(F(" ID: 0x"));
            _debugSerial->print(packetBuf.payload[1], HEX);
            _debugSerial->print(F(" len: "));
            _debugSerial->println(packetBuf.len);
          }
#endif
        }
      }
    }

    // Divert incoming into the correct buffer
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETACK)
      processUBX(incoming, &packetAck, requestedClass, requestedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
      processUBX(incoming, incomingUBX, requestedClass, requestedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
      processUBX(incoming, &packetBuf, requestedClass, requestedID);
    else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
      processUBX(incoming, &packetAuto, requestedClass, requestedID);

    // Finally, increment the frame counter
    ubxFrameCounter++;
  }
}

// Given a character, file it away into the uxb packet structure
// Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
// The payload portion of the packet can be 100s of bytes but the max array size is packetCfgPayloadSize bytes.
// startingSpot can be set so we only record a subset of bytes within a larger packet.
void SFE_UBLOX_GNSS::processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  // If incomingUBX is a user-defined custom packet, then the payload size could be different to packetCfgPayloadSize.
  // TO DO: update this to prevent an overrun when receiving an automatic message
  //        and the incomingUBX payload size is smaller than packetCfgPayloadSize.
  uint16_t maximum_payload_size;
  if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
    maximum_payload_size = packetCfgPayloadSize;
  else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
  {
    // Calculate maximum payload size once Class and ID have been received
    // (This check is probably redundant as activePacketBuffer can only be SFE_UBLOX_PACKET_PACKETAUTO
    //  when ubxFrameCounter >= 3)
    // if (incomingUBX->counter >= 2)
    //{
    maximum_payload_size = getMaxPayloadSize(incomingUBX->cls, incomingUBX->id);
    if (maximum_payload_size == 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("processUBX: getMaxPayloadSize returned ZERO!! Class: 0x"));
        _debugSerial->print(incomingUBX->cls);
        _debugSerial->print(F(" ID: 0x"));
        _debugSerial->println(incomingUBX->id);
      }
#endif
    }
    //}
    // else
    //  maximum_payload_size = 2;
  }
  else
    maximum_payload_size = 2;

  bool overrun = false;

  // Add all incoming bytes to the rolling checksum
  // Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
  if (incomingUBX->counter < incomingUBX->len + 4)
    addToChecksum(incoming);

  if (incomingUBX->counter == 0)
  {
    incomingUBX->cls = incoming;
  }
  else if (incomingUBX->counter == 1)
  {
    incomingUBX->id = incoming;
  }
  else if (incomingUBX->counter == 2) // Len LSB
  {
    incomingUBX->len = incoming;
  }
  else if (incomingUBX->counter == 3) // Len MSB
  {
    incomingUBX->len |= incoming << 8;
  }
  else if (incomingUBX->counter == incomingUBX->len + 4) // ChecksumA
  {
    incomingUBX->checksumA = incoming;
  }
  else if (incomingUBX->counter == incomingUBX->len + 5) // ChecksumB
  {
    incomingUBX->checksumB = incoming;

    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // We're done! Reset the sentence to being looking for a new start char

    // Validate this sentence
    if ((incomingUBX->checksumA == rollingChecksumA) && (incomingUBX->checksumB == rollingChecksumB))
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid
      _signsOfLife = true;                                  // The checksum is valid, so set the _signsOfLife flag

      // Let's check if the class and ID match the requestedClass and requestedID
      // Remember - this could be a data packet or an ACK packet
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_NACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("processUBX: NACK received: Requested Class: 0x"));
          _debugSerial->print(incomingUBX->payload[0], HEX);
          _debugSerial->print(F(" Requested ID: 0x"));
          _debugSerial->println(incomingUBX->payload[1], HEX);
        }
#endif
      }

      // This is not an ACK and we do not have a complete class and ID match
      // So let's check for an "automatic" message arriving
      else if (checkAutomatic(incomingUBX->cls, incomingUBX->id))
      {
        // This isn't the message we are looking for...
        // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("processUBX: incoming \"automatic\" message: Class: 0x"));
          _debugSerial->print(incomingUBX->cls, HEX);
          _debugSerial->print(F(" ID: 0x"));
          _debugSerial->println(incomingUBX->id, HEX);
        }
#endif
      }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->print(F("Incoming: Size: "));
        _debugSerial->print(incomingUBX->len);
        _debugSerial->print(F(" Received: "));
        printPacket(incomingUBX);

        if (incomingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg now valid"));
        }
        if (packetAck.valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck now valid"));
        }
        if (incomingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg classAndIDmatch"));
        }
        if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck classAndIDmatch"));
        }
      }
#endif

      // We've got a valid packet, now do something with it but only if ignoreThisPayload is false
      if (ignoreThisPayload == false)
      {
        processUBXpacket(incomingUBX);
      }
    }
    else // Checksum failure
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

      // Let's check if the class and ID match the requestedClass and requestedID.
      // This is potentially risky as we are saying that we saw the requested Class and ID
      // but that the packet checksum failed. Potentially it could be the class or ID bytes
      // that caused the checksum error!
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }
      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }

      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        // Drive an external pin to allow for easier logic analyzation
        if (debugPin >= 0)
        {
          digitalWrite((uint8_t)debugPin, LOW);
          delay(10);
          digitalWrite((uint8_t)debugPin, HIGH);
        }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        _debugSerial->print(F("Checksum failed:"));
        _debugSerial->print(F(" checksumA: "));
        _debugSerial->print(incomingUBX->checksumA);
        _debugSerial->print(F(" checksumB: "));
        _debugSerial->print(incomingUBX->checksumB);

        _debugSerial->print(F(" rollingChecksumA: "));
        _debugSerial->print(rollingChecksumA);
        _debugSerial->print(F(" rollingChecksumB: "));
        _debugSerial->print(rollingChecksumB);
        _debugSerial->println();
#endif
      }
    }

    // Now that the packet is complete and has been processed, we need to delete the memory
    // allocated for packetAuto
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
    {
      delete[] payloadAuto; // Created with new[]
      payloadAuto = NULL;   // Redundant?
      packetAuto.payload = payloadAuto;
    }
  }
  else // Load this byte into the payload array
  {
    // If an automatic packet comes in asynchronously, we need to fudge the startingSpot
    uint16_t startingSpot = incomingUBX->startingSpot;
    if (checkAutomatic(incomingUBX->cls, incomingUBX->id))
      startingSpot = 0;
    // Check if this is payload data which should be ignored
    if (ignoreThisPayload == false)
    {
      // Begin recording if counter goes past startingSpot
      if ((incomingUBX->counter - 4) >= startingSpot)
      {
        // Check to see if we have room for this byte
        if (((incomingUBX->counter - 4) - startingSpot) < maximum_payload_size) // If counter = 208, starting spot = 200, we're good to record.
        {
          incomingUBX->payload[(incomingUBX->counter - 4) - startingSpot] = incoming; // Store this byte into payload array
        }
        else
        {
          overrun = true;
        }
      }
    }
  }

  // incomingUBX->counter should never reach maximum_payload_size + class + id + len[2] + checksum[2]
  if (overrun || ((incomingUBX->counter == maximum_payload_size + 6) && (ignoreThisPayload == false)))
  {
    // Something has gone very wrong
    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being looking for a new start char
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      if (overrun)
        _debugSerial->print(F("processUBX: buffer overrun detected!"));
      else
        _debugSerial->print(F("processUBX: counter hit maximum_payload_size + 6!"));
      _debugSerial->print(F(" activePacketBuffer: "));
      _debugSerial->print(activePacketBuffer);
      _debugSerial->print(F(" maximum_payload_size: "));
      _debugSerial->println(maximum_payload_size);
    }
#endif
  }

  // Increment the counter
  incomingUBX->counter++;
}

// Once a packet has been received and validated, identify this packet's class/id and update internal flags
void SFE_UBLOX_GNSS::processUBXpacket(ubxPacket *msg)
{
  switch (msg->cls)
  {
  case UBX_CLASS_NAV:
    if (msg->id == UBX_NAV_PVT && msg->len == UBX_NAV_PVT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVT != NULL)
      {
        packetUBXNAVPVT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVT->data.year = extractInt(msg, 4);
        packetUBXNAVPVT->data.month = extractByte(msg, 6);
        packetUBXNAVPVT->data.day = extractByte(msg, 7);
        packetUBXNAVPVT->data.hour = extractByte(msg, 8);
        packetUBXNAVPVT->data.min = extractByte(msg, 9);
        packetUBXNAVPVT->data.sec = extractByte(msg, 10);
        packetUBXNAVPVT->data.valid.all = extractByte(msg, 11);
        packetUBXNAVPVT->data.tAcc = extractLong(msg, 12);
        packetUBXNAVPVT->data.nano = extractSignedLong(msg, 16); // Includes milliseconds
        packetUBXNAVPVT->data.fixType = extractByte(msg, 20);
        packetUBXNAVPVT->data.flags.all = extractByte(msg, 21);
        packetUBXNAVPVT->data.flags2.all = extractByte(msg, 22);
        packetUBXNAVPVT->data.numSV = extractByte(msg, 23);
        packetUBXNAVPVT->data.lon = extractSignedLong(msg, 24);
        packetUBXNAVPVT->data.lat = extractSignedLong(msg, 28);
        packetUBXNAVPVT->data.height = extractSignedLong(msg, 32);
        packetUBXNAVPVT->data.hMSL = extractSignedLong(msg, 36);
        packetUBXNAVPVT->data.hAcc = extractLong(msg, 40);
        packetUBXNAVPVT->data.vAcc = extractLong(msg, 44);
        packetUBXNAVPVT->data.velN = extractSignedLong(msg, 48);
        packetUBXNAVPVT->data.velE = extractSignedLong(msg, 52);
        packetUBXNAVPVT->data.velD = extractSignedLong(msg, 56);
        packetUBXNAVPVT->data.gSpeed = extractSignedLong(msg, 60);
        packetUBXNAVPVT->data.headMot = extractSignedLong(msg, 64);
        packetUBXNAVPVT->data.sAcc = extractLong(msg, 68);
        packetUBXNAVPVT->data.headAcc = extractLong(msg, 72);
        packetUBXNAVPVT->data.pDOP = extractInt(msg, 76);
        packetUBXNAVPVT->data.flags3.all = extractByte(msg, 78);
        packetUBXNAVPVT->data.headVeh = extractSignedLong(msg, 84);
        packetUBXNAVPVT->data.magDec = extractSignedInt(msg, 88);
        packetUBXNAVPVT->data.magAcc = extractInt(msg, 90);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVT->callbackData->iTOW, &packetUBXNAVPVT->data.iTOW, sizeof(UBX_NAV_PVT_data_t));
          packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_TIMEUTC && msg->len == UBX_NAV_TIMEUTC_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVTIMEUTC != NULL)
      {
        packetUBXNAVTIMEUTC->data.iTOW = extractLong(msg, 0);
        packetUBXNAVTIMEUTC->data.tAcc = extractLong(msg, 4);
        packetUBXNAVTIMEUTC->data.nano = extractSignedLong(msg, 8);
        packetUBXNAVTIMEUTC->data.year = extractInt(msg, 12);
        packetUBXNAVTIMEUTC->data.month = extractByte(msg, 14);
        packetUBXNAVTIMEUTC->data.day = extractByte(msg, 15);
        packetUBXNAVTIMEUTC->data.hour = extractByte(msg, 16);
        packetUBXNAVTIMEUTC->data.min = extractByte(msg, 17);
        packetUBXNAVTIMEUTC->data.sec = extractByte(msg, 18);
        packetUBXNAVTIMEUTC->data.valid.all = extractByte(msg, 19);

        // Mark all datums as fresh (not read before)
        packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVTIMEUTC->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVTIMEUTC->callbackData->iTOW, &packetUBXNAVTIMEUTC->data.iTOW, sizeof(UBX_NAV_TIMEUTC_data_t));
          packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_CLOCK && msg->len == UBX_NAV_CLOCK_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVCLOCK != NULL)
      {
        packetUBXNAVCLOCK->data.iTOW = extractLong(msg, 0);
        packetUBXNAVCLOCK->data.clkB = extractSignedLong(msg, 4);
        packetUBXNAVCLOCK->data.clkD = extractSignedLong(msg, 8);
        packetUBXNAVCLOCK->data.tAcc = extractLong(msg, 12);
        packetUBXNAVCLOCK->data.fAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVCLOCK->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVCLOCK->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVCLOCK->callbackData->iTOW, &packetUBXNAVCLOCK->data.iTOW, sizeof(UBX_NAV_CLOCK_data_t));
          packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVCLOCK->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  }
}

// PRIVATE: Add theBytes to the file buffer
bool SFE_UBLOX_GNSS::storeFileBytes(uint8_t *theBytes, uint16_t numBytes)
{
  // First, check that the file buffer has been created
  if ((ubxFileBuffer == NULL) || (fileBufferSize == 0))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("storeFileBytes: file buffer not available!"));
    }
#endif
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  if (numBytes > fileBufferSpaceAvailable())
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("storeFileBytes: insufficient space available! Data will be lost!"));
    }
#endif
    return (false);
  }

  // There is room for all the data in the buffer so copy the data into the buffer
  writeToFileBuffer(theBytes, numBytes);

  return (true);
}

// PRIVATE: Write theBytes to the file buffer
void SFE_UBLOX_GNSS::writeToFileBuffer(uint8_t *theBytes, uint16_t numBytes)
{
  // Start writing at fileBufferHead. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = fileBufferSize - fileBufferHead; // How much space is available 'above' Head?
  if (bytesBeforeWrapAround > numBytes)                             // Is there enough room for all the data?
  {
    bytesBeforeWrapAround = numBytes; // There is enough room for all the data
  }
  memcpy(&ubxFileBuffer[fileBufferHead], theBytes, bytesBeforeWrapAround); // Copy the data into the buffer

  // Is there any data leftover which we need to copy to the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&ubxFileBuffer[0], &theBytes[bytesBeforeWrapAround], bytesLeftToCopy); // Copy the remaining data into the buffer
    fileBufferHead = bytesLeftToCopy;                                             // Update Head. The next byte written will be written here.
  }
  else
  {
    fileBufferHead += numBytes; // Only update Head. The next byte written will be written here.
  }

  // Update fileBufferMaxAvail if required
  uint16_t bytesInBuffer = fileBufferSpaceUsed();
  if (bytesInBuffer > fileBufferMaxAvail)
    fileBufferMaxAvail = bytesInBuffer;
}

// PRIVATE: Check how much space is used in the buffer
uint16_t SFE_UBLOX_GNSS::fileBufferSpaceUsed(void)
{
  if (fileBufferHead >= fileBufferTail) // Check if wrap-around has occurred
  {
    // Wrap-around has not occurred so do a simple subtraction
    return (fileBufferHead - fileBufferTail);
  }
  else
  {
    // Wrap-around has occurred so do a simple subtraction but add in the fileBufferSize
    return ((uint16_t)(((uint32_t)fileBufferHead + (uint32_t)fileBufferSize) - (uint32_t)fileBufferTail));
  }
}
