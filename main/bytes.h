#ifndef __EUROBIN_IOT_BYTES_H
#define __EUROBIN_IOT_BYTES_H
namespace eurobin_iot {
    namespace bytes {
        inline uint16_t bswap(byte b[]) {
        // Big Endian unsigned short to little endian unsigned short
        uint16_t val = ((b[0] << 8) & b[1]);
        return val;
        }

        inline uint16_t makeuint16(int lsb, int msb) {
            return ((msb & 0xFF) << 8) | (lsb & 0xFF);
        }
    }
}

#endif