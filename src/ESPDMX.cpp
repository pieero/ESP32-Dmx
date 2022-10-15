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

/* ----- LIBRARIES ----- */
#include <Arduino.h>
#include <mutex>

#include "ESPDMX.h"

#define DMXSPEED 250000
#define DMXFORMAT SERIAL_8N2
#define BREAKSPEED 83333
#define BREAKFORMAT SERIAL_8N1

void DMXESPSerial::init(HardwareSerial *serial, int chanQuant, uint8_t sendPin)
{
  this->_serial = serial;
  if (chanQuant > 512 || chanQuant <= 0)
  {
    chanQuant = 512;
  }

  this->_numberOfChannels = chanQuant;
  this->_serial->begin(DMXSPEED);
  this->_sendPin = sendPin;
  if (sendPin != 0)
  {
    pinMode(this->_sendPin, OUTPUT);
  }
  this->_dmxStarted = true;
}

uint8_t DMXESPSerial::read(int channel)
{
  if (!this->_dmxStarted)
    return 0;

  if (channel < 1)
  {
    channel = 1;
  }
  else if (channel > 512)
  {
    channel = 512;
  }
  std::lock_guard<std::mutex> lck(this->_dataMutex);
  return this->_dmxData[channel];
}

bool DMXESPSerial::write(int channel, uint8_t value)
{
  if (!this->_dmxStarted)
  {
    return false;
  }
  else if (channel < 1 || channel > this->_numberOfChannels)
  {
    return false;
  }
  else if (value < 0 || value > 255)
  {
    return false;
  }

  std::lock_guard<std::mutex> lck(this->_dataMutex);
  this->_dmxData[channel] = value;
  return true;
}

void DMXESPSerial::end()
{
  std::lock_guard<std::mutex> lck(this->_dataMutex);
  // Reset data to 0
  for (int i = 0; i < this->_numberOfChannels; i++)
  {
    this->_dmxData[i] = 0;
  }

  this->_numberOfChannels = 0;
  this->_serial->end();
  this->_dmxStarted == false;
}

void DMXESPSerial::update()
{
  if (!this->_dmxStarted)
    return;

  // Send break
  if (this->_sendPin != 0)
  {
    digitalWrite(this->_sendPin, HIGH);
  }
  this->_serial->begin(BREAKSPEED, BREAKFORMAT);
  this->_serial->write(0);
  this->_serial->flush();
  delay(1);
  this->_serial->end();

  // Lock data while sending
  std::lock_guard<std::mutex> lck(this->_dataMutex);

  // send data
  this->_serial->begin(DMXSPEED, DMXFORMAT);
  if (this->_sendPin != 0)
  {
    digitalWrite(this->_sendPin, LOW);
  }
  this->_serial->write(this->_dmxData, this->_numberOfChannels);
  this->_serial->flush();
  delay(1);
  this->_serial->end();
}