#pragma once

#include "RingIndex.h"
#include <inttypes.h>

#ifndef RING_BUFFER_CAPACITY
#define RING_BUFFER_CAPACITY 16
#endif

class TRingBuffer {
    protected:
    uint8_t data[RING_BUFFER_CAPACITY]; // Буфер
    TRingIndex head{RING_BUFFER_CAPACITY};       // Кольцевой индексатор
    TRingIndex tail{RING_BUFFER_CAPACITY};       // Кольцевой индексатор

    public:
    inline uint8_t capacity () {
        return static_cast<uint8_t>(RING_BUFFER_CAPACITY) - 1;
    }

    inline bool IsEmpty() {
        return head.idx == tail.idx;
    }

    inline bool IsFull() {
        return head.CalcFwd(1) == tail.idx;
    }

    inline uint8_t size() {
        return (head.idx + (static_cast<uint8_t>(RING_BUFFER_CAPACITY) - tail.idx)) % static_cast<uint8_t>(RING_BUFFER_CAPACITY);
    }

    inline uint8_t FreeSpace() {
        return capacity() - size();
    }

    inline bool Put(uint8_t ch) {
        if (IsFull())
            return false;
        data[head.idx] = ch;
        head.Fwd(1);
        return true;
    }

    inline bool Get(uint8_t& ch) {
        if (IsEmpty())
            return false;
        ch = data[tail.idx];
        tail.Fwd(1);
        return true;
    }

    inline uint8_t& PeekT (uint8_t idx) {
        return data[tail.CalcFwd(idx)];
    }

    inline uint8_t& PeekH (uint8_t idx) {
        return data[head.CalcFwd(idx)];
    }

    inline uint8_t& operator[] (uint8_t idx) {
        return PeekT(idx);
    }
};
