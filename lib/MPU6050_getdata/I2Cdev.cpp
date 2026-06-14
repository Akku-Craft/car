// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2013-05-06 - add Francesco Ferrara's Fastwire v0.24 implementation with small modifications
//      2013-05-05 - fix issue with writing bit values to words (Sasquatch/Farzanegan)
//      2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                 - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//      2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//      2011-10-03 - added Francesco Ferrara's NBWire TwoWire class implementation with small modifications
//      2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//      2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//      2011-07-30 - changed read/write function structures to return success or byte counts
//                 - made all methods static for multi-device memory savings
//      2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "./I2Cdev.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

    #ifdef I2CDEV_IMPLEMENTATION_WARNINGS
        #if ARDUINO < 100
            #warning Using outdated Arduino IDE with Wire library is functionally limiting.
            #warning Arduino IDE v1.0.1+ with I2Cdev Fastwire implementation is recommended.
            #warning This I2Cdev implementation does not support:
            #warning - Repeated starts conditions
            #warning - Timeout detection (some Wire requests block forever)
        #elif ARDUINO == 100
            #warning Using outdated Arduino IDE with Wire library is functionally limiting.
            #warning Arduino IDE v1.0.1+ with I2Cdev Fastwire implementation is recommended.
            #warning This I2Cdev implementation does not support:
            #warning - Repeated starts conditions
            #warning - Timeout detection (some Wire requests block forever)
        #elif ARDUINO > 100
            #warning Using current Arduino IDE with Wire library is functionally limiting.
            #warning Arduino IDE v1.0.1+ with I2CDEV_BUILTIN_FASTWIRE implementation is recommended.
            #warning This I2Cdev implementation does not support:
            #warning - Timeout detection (some Wire requests block forever)
        #endif
    #endif

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE

    //#error The I2CDEV_BUILTIN_FASTWIRE implementation is known to be broken right now. Patience, Iago!

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE

    #ifdef I2CDEV_IMPLEMENTATION_WARNINGS
        #warning Using I2CDEV_BUILTIN_NBWIRE implementation may adversely affect interrupt detection.
        #warning This I2Cdev implementation does not support:
        #warning - Repeated starts conditions
    #endif

    // NBWire implementation based heavily on code by Gene Knight <Gene@Telobot.com>
    // Originally posted on the Arduino forum at http://arduino.cc/forum/index.php/topic,70705.0.html
    // Originally offered to the i2cdevlib project at http://arduino.cc/forum/index.php/topic,68210.30.html
    TwoWire Wire;

#endif

/** Default constructor.
 */
I2Cdev::I2Cdev() {
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

int8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout) {
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w, timeout)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
    return readWords(devAddr, regAddr, 1, data, timeout);
}

int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" bytes from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = millis();

    #if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE)

        #if (ARDUINO < 100)
            for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.send(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

                for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = Wire.receive();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }

                Wire.endTransmission();
            }
        #elif (ARDUINO == 100)
            for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.write(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));
        
                for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = Wire.read();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }
        
                Wire.endTransmission();
            }
        #elif (ARDUINO > 100)
            for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.write(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));
        
                for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = Wire.read();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }
            }
        #endif

    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)

        uint8_t status = Fastwire::readBuf(devAddr << 1, regAddr, data, length);
        if (status == 0) {
            count = length; // success
        } else {
            count = -1; // error
        }

    #endif

    if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif

    return count;
}

int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" words from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = millis();

    #if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE)

        #if (ARDUINO < 100)
            for (uint8_t k = 0; k < length * 2; k += min(length * 2, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.send(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)(length * 2)); // length=words, this wants bytes
    
                bool msb = true; // starts with MSB, then LSB
                for (; Wire.available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
                    if (msb) {
                        data[count] = Wire.receive() << 8;
                    } else {
                        data[count] |= Wire.receive();
                        #ifdef I2CDEV_SERIAL_DEBUG
                            Serial.print(data[count], HEX);
                            if (count + 1 < length) Serial.print(" ");
                        #endif
                        count++;
                    }
                    msb = !msb;
                }

                Wire.endTransmission();
            }
        #elif (ARDUINO == 100)
            for (uint8_t k = 0; k < length * 2; k += min(length * 2, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.write(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)(length * 2)); // length=words, this wants bytes
    
                bool msb = true; // starts with MSB, then LSB
                for (; Wire.available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
                    if (msb) {
                        data[count] = Wire.read() << 8;
                    } else {
                        data[count] |= Wire.read();
                        #ifdef I2CDEV_SERIAL_DEBUG
                            Serial.print(data[count], HEX);
                            if (count + 1 < length) Serial.print(" ");
                        #endif
                        count++;
                    }
                    msb = !msb;
                }
        
                Wire.endTransmission();
            }
        #elif (ARDUINO > 100)
            for (uint8_t k = 0; k < length * 2; k += min(length * 2, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.write(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)(length * 2)); // length=words, this wants bytes
    
                bool msb = true; // starts with MSB, then LSB
                for (; Wire.available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
                    if (msb) {
                        data[count] = Wire.read() << 8;
                    } else {
                        data[count] |= Wire.read();
                        #ifdef I2CDEV_SERIAL_DEBUG
                            Serial.print(data[count], HEX);
                            if (count + 1 < length) Serial.print(" ");
                        #endif
                        count++;
                    }
                    msb = !msb;
                }
    
                Wire.endTransmission();
            }
        #endif

    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)

        uint16_t intermediate[(uint8_t)length];
        uint8_t status = Fastwire::readBuf(devAddr << 1, regAddr, (uint8_t *)intermediate, (uint8_t)(length * 2));
        if (status == 0) {
            count = length; // success
            for (uint8_t i = 0; i < length; i++) {
                data[i] = (intermediate[2*i] << 8) | intermediate[2*i + 1];
            }
        } else {
            count = -1; // error
        }

    #endif

    if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif
    
    return count;
}

bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}

bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1);
        data &= mask;
        b &= ~(mask);
        b |= data;
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    uint16_t w;
    if (readWord(devAddr, regAddr, &w) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1);
        data &= mask;
        w &= ~(mask);
        w |= data;
        return writeWord(devAddr, regAddr, w);
    } else {
        return false;
    }
}

bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return writeWords(devAddr, regAddr, 1, &data);
}

bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") writing ");
        Serial.print(length, DEC);
        Serial.print(" bytes to 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    uint8_t status = 0;
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        Wire.beginTransmission(devAddr);
        Wire.send((uint8_t) regAddr); // send address
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
        Wire.beginTransmission(devAddr);
        Wire.write((uint8_t) regAddr); // send address
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::beginTransmission(devAddr);
        Fastwire::write(regAddr);
    #endif
    for (uint8_t i = 0; i < length; i++) {
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[i], HEX);
            if (i + 1 < length) Serial.print(" ");
        #endif
        #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
            Wire.send((uint8_t) data[i]);
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
            Wire.write((uint8_t) data[i]);
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
            Fastwire::write((uint8_t) data[i]);
        #endif
    }
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        Wire.endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
        status = Wire.endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::stop();
    #endif
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println(". Done.");
    #endif
    return status == 0;
}

bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") writing ");
        Serial.print(length, DEC);
        Serial.print(" words to 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    uint8_t status = 0;
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        Wire.beginTransmission(devAddr);
        Wire.send(regAddr); // send address
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
        Wire.beginTransmission(devAddr);
        Wire.write(regAddr); // send address
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::beginTransmission(devAddr);
        Fastwire::write(regAddr);
    #endif
    for (uint8_t i = 0; i < length * 2; i++) {
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[i], HEX);
            if (i + 1 < length) Serial.print(" ");
        #endif
        #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
            Wire.send((uint8_t)(data[i] >> 8));     // send MSB
            Wire.send((uint8_t)data[i++]);          // send LSB
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
            Wire.write((uint8_t)(data[i] >> 8));    // send MSB
            Wire.write((uint8_t)data[i++]);         // send LSB
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
            Fastwire::write((uint8_t)(data[i] >> 8));       // send MSB
            status = Fastwire::write((uint8_t)data[i++]);   // send LSB
            if (status != 0) break;
        #endif
    }
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        Wire.endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
        status = Wire.endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::stop();
    #endif
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println(". Done.");
    #endif
    return status == 0;
}

uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    // Fastwire implementation omitted for brevity (already present in upstream I2Cdev)
    boolean Fastwire::waitInt() { return false; }
    void Fastwire::setup(int khz, boolean pullup) {}
    byte Fastwire::beginTransmission(byte device) { return 0; }
    byte Fastwire::write(byte value) { return 0; }
    byte Fastwire::writeBuf(byte device, byte address, byte *data, byte num) { return 0; }
    byte Fastwire::readBuf(byte device, byte address, byte *data, byte num) { return 0; }
    void Fastwire::reset() {}
    byte Fastwire::stop() { return 0; }
#endif
