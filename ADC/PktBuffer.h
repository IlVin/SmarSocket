#pragma once

#ifndef RING_BUFFER_CAPACITY
#define RING_BUFFER_CAPACITY (4 * 16 + 1)
#endif

#include "RingBuffer.h"
#include <inttypes.h>

const uint16_t CS_CONST = 44111;

class TPktBuffer: public TRingBuffer {
    public:
    inline uint8_t CheckSum(uint8_t pktId, uint8_t dataHi, uint8_t dataLo) const {
        return static_cast<uint8_t>(
            static_cast<uint16_t>(pktId) * CS_CONST +
            static_cast<uint16_t>(dataHi) * CS_CONST +
            static_cast<uint16_t>(dataLo) * CS_CONST
        );
    }

    inline bool PutPkt(uint8_t idType, uint8_t idPin, uint16_t data) {
        if (FreeSpace() >= 4) {
            uint8_t pktId = (idType << 4) | (idPin & 0b00001111) | 0b10000000;
            uint8_t dataHi = static_cast<uint8_t>((data >> 7) & 0b01111111);
            uint8_t dataLo = static_cast<uint8_t>(data & 0b01111111);
            PeekH(0) = pktId;
            PeekH(1) = dataHi;
            PeekH(2) = dataLo;
            PeekH(3) = CheckSum(pktId, dataHi, dataLo);
            head.Fwd(4);
            return true;
        }
        return false;
    }

    bool GetPkt(uint8_t& idType, uint8_t& idPin, uint16_t& data) {
        while (size() >= 4) {
            uint8_t pktId = PeekT(0);
            uint8_t dataHi = PeekT(1);
            uint8_t dataLo = PeekT(2);
            uint8_t cs = PeekT(3);
            if (   ((pktId  & 0b10000000) != 0)
                && ((dataHi & 0b10000000) == 0)
                && ((dataLo & 0b10000000) == 0)
                && (cs == CheckSum(pktId, dataHi, dataLo))
            ) {
                idType = (pktId >> 4) & 0b00000111;
                idPin  = pktId & 0b00001111;
                data   = (static_cast<uint16_t>(dataHi) << 7) | static_cast<uint16_t>(dataLo);
                tail.Fwd(4);
                return true;
            }
            tail.Fwd(1);
        }
        return false;
    }
};
