// Derived from Adafruit's FT6236 lib
// https://github.com/DustinWatts/FT6236
// Original license below:
/*
This is a library for the FT6236 touchscreen controller by FocalTech.
The FT6236 and FT6236u work the same way.
A lot of this library is originally written by Limor Fried/Ladyada.
Because Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!
@section author Author
Written by Limor Fried/Ladyada for Adafruit Industries.
@section license License
MIT license, all text above must be included in any redistribution
*/
// Portions derived from FT6X36-IDF https://github.com/martinberlin/FT6X36-IDF
// Original license below:
/*
MIT License

Copyright (c) 2019 strange_v

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#pragma once
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
namespace arduino {
#else
#include <esp_timer.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
namespace esp_idf {
#endif

template <uint16_t Width, uint16_t Height, uint8_t Threshhold = 128, uint8_t Address = 0x38>
class ft6336 final {
    constexpr static const uint8_t TOUCH_REG_XL = 0x04;
    constexpr static const uint8_t TOUCH_REG_XH = 0x03;
    constexpr static const uint8_t TOUCH_REG_YL = 0x06;
    constexpr static const uint8_t TOUCH_REG_YH = 0x05;
    constexpr static const uint8_t TOUCH_REG_NUMTOUCHES = 0x2;
    constexpr static const uint8_t TOUCH_REG_THRESHHOLD = 0x80;
    constexpr static const uint8_t TOUCH_REG_VENDID = 0xA8;
    constexpr static const uint8_t TOUCH_REG_CHIPID = 0xA3;
    constexpr static const uint8_t FT6336_VENDID = 0x11;
    constexpr static const uint8_t FT6206_CHIPID = 0x6;
    constexpr static const uint8_t FT6336_CHIPID = 0x64;
#ifdef ARDUINO
    TwoWire& m_i2c;
#else
    constexpr static const uint8_t ACK_CHECK_EN = 0x1;
    constexpr static const uint8_t ACK_CHECK_DIS = 0x0;
    constexpr static const uint8_t ACK_VAL = 0x0;
    constexpr static const uint8_t NACK_VAL = 0x1;
    i2c_port_t m_i2c;
#endif
    uint8_t m_rotation;
    bool m_initialized;
    size_t m_touches;
    uint16_t m_touches_x[2], m_touches_y[2], m_touches_id[2];

    ft6336(const ft6336& rhs) = delete;
    ft6336& operator=(const ft6336& rhs) = delete;
    void do_move(ft6336& rhs) {
        m_i2c = rhs.m_i2c;
        m_rotation = rhs.m_rotation;
        m_initialized = rhs.m_initialized;
        m_touches = rhs.m_touches;
        memcpy(m_touches_x, rhs.m_touches_x, sizeof(m_touches_x));
        memcpy(m_touches_y, rhs.m_touches_y, sizeof(m_touches_y));
        memcpy(m_touches_id, rhs.m_touches_id, sizeof(m_touches_id));
    }
    int reg(int r) const {
#ifdef ARDUINO
        int result = 0;
        m_i2c.beginTransmission(address);
        m_i2c.write(r);
        m_i2c.endTransmission();
        m_i2c.requestFrom((uint8_t)address, (uint8_t)1);
        if (m_i2c.available()) {
            result = m_i2c.read();
        }
        return result;
#else
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, r, I2C_MASTER_ACK);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
        uint8_t result;
        i2c_master_read_byte(cmd, &result, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(m_i2c, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        return result;
#endif
    }
    void reg(int r, int value) {
#ifdef ARDUINO
        m_i2c.beginTransmission(address);
        m_i2c.write((uint8_t)r);
        m_i2c.write((uint8_t)value);
        m_i2c.endTransmission();
#else
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, r, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(m_i2c, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
#endif
    }
    void read_all() {
        uint8_t i2cdat[16];
#ifdef ARDUINO
        m_i2c.beginTransmission(address);
        m_i2c.write((uint8_t)0);
        m_i2c.endTransmission();

        m_i2c.requestFrom((uint8_t)address, (uint8_t)16);
        for (uint8_t i = 0; i < 16; i++)
            i2cdat[i] = m_i2c.read();
#else
    // Read data
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address<<1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(m_i2c, cmd, pdMS_TO_TICKS( 1000));
    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address<<1)|1, ACK_CHECK_EN);
    i2c_master_read(cmd, i2cdat, sizeof(i2cdat),  I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(m_i2c, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
#endif
        m_touches = i2cdat[0x02];
        if (m_touches > 2) {
            m_touches = 0;
        }

        for (uint8_t i = 0; i < 2; i++) {
            m_touches_x[i] = i2cdat[0x03 + i * 6] & 0x0F;
            m_touches_x[i] <<= 8;
            m_touches_x[i] |= i2cdat[0x04 + i * 6];
            m_touches_y[i] = i2cdat[0x05 + i * 6] & 0x0F;
            m_touches_y[i] <<= 8;
            m_touches_y[i] |= i2cdat[0x06 + i * 6];
            m_touches_id[i] = i2cdat[0x05 + i * 6] >> 4;
        }
    }
    bool read_point(size_t n, uint16_t* out_x, uint16_t* out_y) const {
        if (m_touches == 0 || n >= m_touches) {
            if (out_x != nullptr) {
                *out_x = 0;
            }
            if (out_y != nullptr) {
                *out_y = 0;
            }
            return false;
        }
        uint16_t x = m_touches_x[n];
        uint16_t y = m_touches_y[n];
        if (x >= native_width) {
            x = native_width - 1;
        }
        if (y >= native_height) {
            y = native_height - 1;
        }
        translate(x, y);
        if (out_x != nullptr) {
            *out_x = x;
        }
        if (out_y != nullptr) {
            *out_y = y;
        }
        return true;
    }
    void translate(uint16_t& x, uint16_t& y) const {
        uint16_t tmp;
        switch (m_rotation & 3) {
            case 1:
                tmp = x;
                x = y;
                y = native_width - tmp - 1;
                break;
            case 2:
                x = native_width - x - 1;
                y = native_height - y - 1;
                break;
            case 3:
                tmp = x;
                x = native_height - y - 1;
                y = tmp;
            default:
                break;
        }
    }

   public:
    constexpr static const uint16_t native_width = Width;
    constexpr static const uint16_t native_height = Height;
    constexpr static const uint8_t threshhold = Threshhold;
    constexpr static const uint8_t address = Address;
    ft6336(ft6336&& rhs) {
        do_move(rhs);
    }
    ft6336& operator=(ft6336&& rhs) {
        do_move(rhs);
        return *this;
    }
    ft6336(
#ifdef ARDUINO
        TwoWire& i2c = Wire
#else
        i2c_port_t i2c = I2C_NUM_0
#endif
        ) : m_i2c(i2c), m_rotation(0), m_touches(0) {
    }

    bool initialized() const {
        return m_initialized;
    }
    bool initialize() {
        if (!m_initialized) {
#ifdef ARDUINO
            m_i2c.begin();
#endif
            reg(TOUCH_REG_THRESHHOLD, threshhold);

            // Check if our chip has the correct Vendor ID
            /*if (reg(TOUCH_REG_VENDID) != FT6336_VENDID) {
                return false;
            }
            // Check if our chip has the correct Chip ID.
            uint8_t id = reg(TOUCH_REG_CHIPID);
            if ((id != FT6336_CHIPID)) {
                return false;
            }*/
            m_touches = 0;
            m_initialized = true;
        }
        return m_initialized;
    }
    uint8_t rotation() const {
        return m_rotation;
    }
    void rotation(uint8_t value) {
        m_rotation = value & 3;
    }
    uint16_t width() const {
        return m_rotation & 1 ? native_height : native_width;
    }
    uint16_t height() const {
        return m_rotation & 1 ? native_width : native_height;
    }
    size_t touches() const {
        uint8_t result = reg(TOUCH_REG_NUMTOUCHES);
        if (result > 2) {
            result = 0;
        }
        return result;
    }
    bool xy(uint16_t* out_x, uint16_t* out_y) const {
        return read_point(0, out_x, out_y);
    }
    bool xy2(uint16_t* out_x, uint16_t* out_y) const {
        return read_point(1, out_x, out_y);
    }
    bool update() {
        if (!initialize()) {
            return false;
        }
        read_all();
        return true;
    }
};
}  // namespace arduino