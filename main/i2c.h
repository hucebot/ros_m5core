#ifndef __EUROBIN_IOT_I2C_H
#define __EUROBIN_IOT_I2C_H

namespace eurobin_iot {
    namespace i2c {

        // return 0 if no error
        inline uint8_t test_address(uint8_t address) {
            Wire.beginTransmission(address);
            uint8_t error = Wire.endTransmission();
            return error;
        }

        inline void write_byte_data(uint8_t address, uint8_t data) {
            Wire.beginTransmission(address);
            Wire.write(data);
            Wire.endTransmission();
        }

        inline void write_byte_data_at(uint8_t address, uint8_t reg, byte data) {
            // write data word at address and register
            Wire.beginTransmission(address);
            Wire.write(reg);
            Wire.write(data);
            Wire.endTransmission();
        }

        inline void write_word_data_at(uint8_t address, uint8_t reg, uint16_t data) {
            // write data word at address and register
            byte b0 = (data & 0xFF);
            byte b1 = ((data >> 8) && 0xFF);

            Wire.beginTransmission(address);
            Wire.write(reg);
            Wire.write(b0);
            Wire.write(b1);
            Wire.endTransmission();
        }

        inline uint8_t read_byte_data(uint8_t address) {
            Wire.requestFrom(address, (uint8_t) 1);
            while (Wire.available() < 1) delay(1);
            byte b = Wire.read();
            return b;
        }

        inline uint8_t read_byte_data_at(uint8_t address, uint8_t reg) {
            //x`write_byte_data((byte)0x00);
            write_byte_data(address, reg);
            Wire.requestFrom(address, (uint8_t)1);
            while (Wire.available() < 1) delay(1);
            byte b = Wire.read();

            return b;
        }

        inline uint8_t* read_block_data_at(uint8_t address, uint8_t reg, uint8_t sz) {
            static uint8_t buffer[16]; // NOT thread-safe

            int i = 0;
            write_byte_data(address, reg);
            Wire.requestFrom(address, sz);
            for (i = 0; i < sz; i++) {
                while (Wire.available() < 1) delay(1);
                buffer[i] = Wire.read();
            }
            return buffer;
        }
    }
}
#endif