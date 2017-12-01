#pragma once

#include <inttypes.h>

class TRingIndex {
    private:
    uint8_t sz;

    public:
    volatile uint8_t idx;

    TRingIndex (uint8_t size): sz(size), idx(0) { }

    ~TRingIndex() = default;

    inline uint8_t size() {
        return sz;
    }

    inline uint8_t CalcFwd (const uint8_t& fwd = 1) {
        return (idx + fwd) % sz;
    }

    inline void Fwd (const uint8_t& fwd = 1) {
        idx = CalcFwd(fwd);
    }
};

