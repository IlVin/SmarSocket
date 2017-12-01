#pragma once

#ifndef RING_BUFFER_CAPACITY
#define RING_BUFFER_CAPACITY 3 * 16
#endif

#include "RingBuffer.h"
#include <inttypes.h>

class TPktBuffer: public TRingBuffer {
    public:
    inline bool PutPkt(const uint8_t& idType, const uint8_t& idPin, const uint16_t& data ) {
        if (FreeSpace() >= 3) {
            PeekH(0) = (idType << 4) | (idPin & 0b00001111) | 0b10000000;
            PeekH(1) = static_cast<uint8_t>((data >> 7) & 0b01111111);
            PeekH(2) = static_cast<uint8_t>(data & 0b01111111);
            head.Fwd(3);
            return true;
        }
        return false;
    }

    bool GetPkt(uint8_t& idType, uint8_t& idPin, uint16_t& data ) {
        while (size() >= 3) {
            uint8_t pktId = PeekT(0);
            uint8_t dataHi = PeekT(1);
            uint8_t dataLo = PeekT(2);
            if (   ((pktId  & 0b10000000) != 0)
                && ((dataHi & 0b10000000) == 0)
                && ((dataLo & 0b10000000) == 0)
            ) {
                idType = (pktId >> 4) & 0b00000111;
                idPin  = pktId & 0b00001111;
                data   = (static_cast<uint16_t>(dataHi) << 7) | static_cast<uint16_t>(dataLo);
                tail.Fwd(3);
                return true;
            }
            tail.Fwd(1);
        }
        return false;
    }
};
