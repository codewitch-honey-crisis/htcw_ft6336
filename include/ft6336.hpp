#pragma once
#include <Arduino.h>
#include <Wire.h>
namespace arduino {
template <uint16_t Width, uint16_t Height, int8_t PinInt, uint8_t Address = 0x38>
class ft6336 final {
    constexpr static const uint8_t REG_DEVICE_MODE = 0x00;
    constexpr static const uint8_t REG_GESTURE_ID = 0x01;
    constexpr static const uint8_t REG_NUM_TOUCHES = 0x02;
    constexpr static const uint8_t REG_P1_XH = 0x03;
    constexpr static const uint8_t REG_P1_XL = 0x04;
    constexpr static const uint8_t REG_P1_YH = 0x05;
    constexpr static const uint8_t REG_P1_YL = 0x06;
    constexpr static const uint8_t REG_P1_WEIGHT = 0x07;
    constexpr static const uint8_t REG_P1_MISC = 0x08;
    constexpr static const uint8_t REG_P2_XH = 0x09;
    constexpr static const uint8_t REG_P2_XL = 0x0A;
    constexpr static const uint8_t REG_P2_YH = 0x0B;
    constexpr static const uint8_t REG_P2_YL = 0x0C;
    constexpr static const uint8_t REG_P2_WEIGHT = 0x0D;
    constexpr static const uint8_t REG_P2_MISC = 0x0E;
    constexpr static const uint8_t REG_THRESHHOLD = 0x80;
    constexpr static const uint8_t REG_FILTER_COEF = 0x85;
    constexpr static const uint8_t REG_CTRL = 0x86;
    constexpr static const uint8_t REG_TIME_ENTER_MONITOR = 0x87;
    constexpr static const uint8_t REG_TOUCHRATE_ACTIVE = 0x88;
    constexpr static const uint8_t REG_TOUCHRATE_MONITOR = 0x89;  // value in ms
    constexpr static const uint8_t REG_RADIAN_VALUE = 0x91;
    constexpr static const uint8_t REG_OFFSET_LEFT_RIGHT = 0x92;
    constexpr static const uint8_t REG_OFFSET_UP_DOWN = 0x93;
    constexpr static const uint8_t REG_DISTANCE_LEFT_RIGHT = 0x94;
    constexpr static const uint8_t REG_DISTANCE_UP_DOWN = 0x95;
    constexpr static const uint8_t REG_DISTANCE_ZOOM = 0x96;
    constexpr static const uint8_t REG_LIB_VERSION_H = 0xA1;
    constexpr static const uint8_t REG_LIB_VERSION_L = 0xA2;
    constexpr static const uint8_t REG_CHIPID = 0xA3;
    constexpr static const uint8_t REG_INTERRUPT_MODE = 0xA4;
    constexpr static const uint8_t REG_POWER_MODE = 0xA5;
    constexpr static const uint8_t REG_FIRMWARE_VERSION = 0xA6;
    constexpr static const uint8_t REG_PANEL_ID = 0xA8;
    constexpr static const uint8_t REG_STATE = 0xBC;

    constexpr static const uint8_t PMODE_ACTIVE = 0x00;
    constexpr static const uint8_t PMODE_MONITOR = 0x01;
    constexpr static const uint8_t PMODE_STANDBY = 0x02;
    constexpr static const uint8_t PMODE_HIBERNATE = 0x03;

    /* Possible values returned by GEST_ID_REG */
    constexpr static const uint8_t GEST_ID_NO_GESTURE = 0x00;
    constexpr static const uint8_t GEST_ID_MOVE_UP = 0x10;
    constexpr static const uint8_t GEST_ID_MOVE_RIGHT = 0x14;
    constexpr static const uint8_t GEST_ID_MOVE_DOWN = 0x18;
    constexpr static const uint8_t GEST_ID_MOVE_LEFT = 0x1C;
    constexpr static const uint8_t GEST_ID_ZOOM_IN = 0x48;
    constexpr static const uint8_t GEST_ID_ZOOM_OUT = 0x49;

    constexpr static const uint8_t VENDID = 0x11;
    constexpr static const uint8_t FT6206_CHIPID = 0x06;
    constexpr static const uint8_t FT6236_CHIPID = 0x36;
    constexpr static const uint8_t FT6336_CHIPID = 0x64;

    constexpr static const uint8_t DEFAULT_THRESHOLD = 22;
    struct point final {
        uint16_t x;
        uint16_t y;
    };
    TwoWire& m_i2c;
    uint32_t m_last_read_ts;
    uint8_t m_interval;
    point m_points[2];
    bool m_updated;
    bool m_changed;
    uint8_t m_point_count;
    uint8_t m_point_0_finger;
    uint8_t m_rotation;
    uint8_t reg(uint8_t address) const {
        m_i2c.beginTransmission(ft6336::address);
        m_i2c.write(address);
        m_i2c.endTransmission();
        m_i2c.requestFrom((uint8_t)ft6336::address, uint8_t(1));
        return m_i2c.read();
    }

    void reg(uint8_t address, uint8_t value) {
        m_i2c.beginTransmission(ft6336::address);
        m_i2c.write(address);
        m_i2c.write((uint8_t)value);
        m_i2c.endTransmission();
    }

    // Reading size bytes into data
    void reg(uint8_t address, uint8_t size, uint8_t* data) const {
        m_i2c.beginTransmission((uint8_t)ft6336::address);
        m_i2c.write(address);
        m_i2c.endTransmission();
        m_i2c.requestFrom((uint8_t)ft6336::address, size);
        for (uint8_t i = 0; i < size; i++) data[i] = m_i2c.read();
    }
    // TODO: Test this
    static void translate(point& pt,int rotation) {
        uint16_t tmp;
        switch(rotation & 3) {
            case 0:
                tmp = pt.x;
                pt.x = pt.y;
                pt.y = pt.x;
                break;
            case 1:
                break;
            case 2:
                tmp = width-pt.x-1;
                pt.x = pt.y;
                pt.y = pt.x;
                break;
            case 3:
                tmp = pt.x;
                pt.x = height-pt.y-1;
                pt.y = pt.x;
            default:
                break;
            
        }
    }
   public:
    constexpr static const uint16_t width = Width;
    constexpr static const uint16_t height = Height;
    constexpr static const int8_t pin_int = PinInt;
    constexpr static const uint8_t address = Address;
    ft6336(TwoWire& i2c = Wire) : m_i2c(i2c), m_last_read_ts(0), m_interval(0), m_updated(false),m_changed(false),m_point_count(0),m_point_0_finger(0),m_rotation(0) {
        m_points[0].x = m_points[0].y = m_points[1].x = m_points[1].y = 65535;
    }
    inline void interrupt_enabled(bool value) {
        reg(REG_INTERRUPT_MODE, value);
    }
    inline bool interrupt_enabled() const {
        return reg(REG_INTERRUPT_MODE);
    }
    inline void interval(uint8_t ivl) {
        reg(REG_TOUCHRATE_ACTIVE, ivl);
        m_interval = interval();
    }

    inline uint8_t interval() const {
        return reg(REG_TOUCHRATE_ACTIVE);
    }
    inline bool pressed() const {
        return !digitalRead(pin_int) && m_updated && m_changed;
    }
    inline uint8_t rotation() const {
        return m_rotation;
    }
    inline void rotation(uint8_t value) {
        m_rotation = value & 3;
    }
    void xy(uint16_t* out_x, uint16_t* out_y) const {
        if(!m_updated) return;
        if(m_point_count>0) {
            if(out_x!=nullptr) {
                *out_x = m_points[0].x;
            }
            if(out_y!=nullptr) {
                *out_y = m_points[0].y;
            }
        }
    }
    void xy2(uint16_t* out_x, uint16_t* out_y) const {
        if(!m_updated) return;
        if(m_point_count>1) {
            if(out_x!=nullptr) {
                *out_x = m_points[1].x;
            }
            if(out_y!=nullptr) {
                *out_y = m_points[1].y;
            }
        }
    }
    bool update() {
        if (!m_interval) {
            m_interval = interval();
        }
        m_updated=false;
        m_changed = false;
        if (millis() - m_last_read_ts < m_interval) {
            return false;
        }
        m_last_read_ts = millis();
        point p[2];
        uint8_t pts = 0;
        uint8_t p0f = 0;
        if (!digitalRead(pin_int)) {
            uint8_t data[11];
            reg(REG_NUM_TOUCHES, 11, data);
            pts = data[0];
            if (pts > 2) return false;
            if (pts) {
                // Read the data. Never mind trying to read the "weight" and
                // "size" properties or using the built-in gestures: they
                // are always set to zero.
                p0f = (data[3] >> 4) ? 1 : 0;
                p[0].x = ((data[1] << 8) | data[2]) & 0x0fff;
                p[0].y = ((data[3] << 8) | data[4]) & 0x0fff;
                if (p[0].x >= height || p[0].y >= width) return false;
                if (pts == 2) {
                    p[1].x = ((data[7] << 8) | data[8]) & 0x0fff;
                    p[1].y = ((data[9] << 8) | data[10]) & 0x0fff;
                    if (p[1].x >= height || p[1].y >= width) return false;
                }
            }
        }
        translate(p[0],m_rotation);
        translate(p[1],m_rotation);
        if (p[0].x != m_points[0].x || p[1].x != m_points[1].x ||
            p[0].y != m_points[0].y || p[1].y != m_points[1].y) {
            m_changed = true;
            m_points[0] = p[0];
            m_points[1] = p[1];
            m_point_count = pts;
            m_point_0_finger = p0f;
        }
        m_updated = true;
        return true;
    }
};
}  // namespace arduino