// - - - - -
// ESPDMX - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// ESPDMX.cpp: Library implementation file
//
// Copyright (C) 2015  Rick <ricardogg95@gmail.com>
// This work is licensed under a GNU style license.
//
// Last change: Marcel Seerig <https://github.com/mseerig>
//
// Documentation and samples are available at https://github.com/Rickgg/ESP-Dmx
// - - - - -

#include <Arduino.h>
#include <inttypes.h>

#ifndef ESPDMX_h
#define ESPDMX_h

// ---- Methods ----

class DMXESPSerial
{
public:
  /// @brief Initialize the DMX library with serial port and number of channels
  /// @param serial The HardwareSerial port to use (Serial, Serial1 oru Serial2)
  /// @param MaxChan The number of channels. Max: 512
  /// @param sendPin The pin to bring HIGH during break and LOW durin sending. Used to control RS485 send/receiver ICs.
  /// A value of 0 disabled the sendPin functionality.
  void init(HardwareSerial *serial, int MaxChan, uint8_t sendPin);
  /// @brief Read the current value from the send buffer
  /// @param channel The channel to read.
  /// @return The value set in the send buffer. If the library is not initialized a value of 0 is returned.
  uint8_t read(int channel);
  /// @brief Set the value for a given channel in the send buffer
  /// @param channel The channel
  /// @param value The value to send
  /// @return True if successful, oterwise false.
  bool write(int channel, uint8_t value);
  /// @brief Write out all data in the bus
  void update();
  /// @brief Stop sending
  void end();

private:
  /// @brief The serial port to send DMX data on
  HardwareSerial *_serial;
  /// @brief The number of channels to send
  uint16_t _numberOfChannels;
  /// @brief The pin to bright HIGH during break and LOW during sending.
  uint8_t _sendPin;
  /// @brief The buffer to store DMX data
  uint8_t _dmxData[512] = {};
  /// @brief Indicate wether or not the DMX library has been initialized.
  bool _dmxStarted = false;
  /// @brief Mutex to make sure only one instance has access to _dmxData buffer at the same time.
  std::mutex _dataMutex;
};

#endif
