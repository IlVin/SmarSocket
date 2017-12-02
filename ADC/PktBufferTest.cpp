
#define RING_BUFFER_CAPACITY 9
#include "PktBuffer.h"

//#include <iostream>
//#include <string>
#include <assert.h>

int main() {

    uint8_t idType;
    uint8_t idPin;
    uint16_t data;

    TPktBuffer pb {};

    assert(pb.PutPkt(0b00000111, 0b00001111, 0b0011111111111111) == true);
    assert(pb.size() == 4);
    assert(pb[0] == 0b11111111);
    assert(pb[1] == 0b01111111);
    assert(pb[2] == 0b01111111);

    assert(pb.PutPkt(0b00000000, 0b00000000, 0b0000000000000000) == true);
    assert(pb.size() == 8);
    assert(pb[0] == 0b11111111);
    assert(pb[1] == 0b01111111);
    assert(pb[2] == 0b01111111);

    assert(pb[4] == 0b10000000);
    assert(pb[5] == 0b00000000);
    assert(pb[6] == 0b00000000);

    assert(pb.PutPkt(0b00000010, 0b00000010, 0b0000000000000010) == false);

    assert(pb.GetPkt(idType, idPin, data) == true);
    assert(pb.size() == 4);
    assert(pb[0] == 0b10000000);
    assert(pb[1] == 0b00000000);
    assert(pb[2] == 0b00000000);
    assert(idType == 0b00000111);
    assert(idPin  == 0b00001111);
    assert(data   == 0b0011111111111111);

    assert(pb.GetPkt(idType, idPin, data) == true);
    assert(pb.size() == 0);
    assert(idType == 0b00000000);
    assert(idPin  == 0b00000000);
    assert(data   == 0b0000000000000000);

    assert(pb.GetPkt(idType, idPin, data) == false );

    assert(pb.Put(0x00) == true);
    assert(pb.Put(0xFF) == true);
    assert(pb.PutPkt(0b00000111, 0b00001111, 0b0011111111111111) == true);
    assert(pb.size() == 6);
    assert(pb[2] == 0b11111111);
    assert(pb[3] == 0b01111111);
    assert(pb[4] == 0b01111111);

    assert(pb.PutPkt(0b00000010, 0b00000010, 0b0000000000000010) == false);

    assert(pb.GetPkt(idType, idPin, data) == true);
    assert(pb.size() == 0);
    assert(idType == 0b00000111);
    assert(idPin  == 0b00001111);
    assert(data   == 0b0011111111111111);

    assert(pb.GetPkt(idType, idPin, data) == false);
    assert(pb.size() == 0);

    return 0;
}
