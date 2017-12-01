
#define RING_BUFFER_CAPACITY 3
#include "ADC/RingBuffer.h"

#include <iostream>
#include <string>
#include <assert.h>

int main() {

    TRingBuffer rb {};

    assert(rb.capacity() == 3);
    assert(rb.size() == 0);
    assert(rb.IsEmpty() == true);
    assert(rb.IsFull() == false);

    assert(rb.PutC('1') == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 1);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == false);
    assert(rb[0] == '1');

    assert(rb.PutC('2') == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 2);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == false);
    assert(rb[0] == '1');
    assert(rb[1] == '2');

    assert(rb.PutC('3') == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 3);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == true);
    assert(rb[0] == '1');
    assert(rb[1] == '2');
    assert(rb[2] == '3');

    assert(rb.PutC('4') == false);
    assert(rb.capacity() == 3);
    assert(rb.size() == 3);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == true);
    assert(rb[0] == '1');
    assert(rb[1] == '2');
    assert(rb[2] == '3');
    assert(rb[3] == '1');

    uint8_t data;
    assert(rb.GetC(data) == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 2);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == false);
    assert(rb[0] == '2');
    assert(rb[1] == '3');

    assert(rb.GetC(data) == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 1);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == false);
    assert(rb[0] == '3');

    assert(rb.GetC(data) == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 0);
    assert(rb.IsEmpty() == true);
    assert(rb.IsFull() == false);

    assert(rb.GetC(data) == false);
    assert(rb.capacity() == 3);
    assert(rb.size() == 0);
    assert(rb.IsEmpty() == true);
    assert(rb.IsFull() == false);


    return 0;
}
