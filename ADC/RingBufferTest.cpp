
#define RING_BUFFER_CAPACITY 4
#include "RingBuffer.h"

#include <iostream>
#include <string>
#include <assert.h>

int main() {

    TRingBuffer rb {};

    assert(rb.capacity() == 3);
    assert(rb.size() == 0);
    assert(rb.IsEmpty() == true);
    assert(rb.IsFull() == false);

    assert(rb.Put('1') == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 1);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == false);
    assert(rb[0] == '1');

    assert(rb.Put('2') == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 2);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == false);
    assert(rb[0] == '1');
    assert(rb[1] == '2');

    assert(rb.Put('3') == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 3);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == true);
    assert(rb[0] == '1');
    assert(rb[1] == '2');
    assert(rb[2] == '3');

    assert(rb.Put('4') == false);
    assert(rb.capacity() == 3);
    assert(rb.size() == 3);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == true);
    assert(rb[0] == '1');
    assert(rb[1] == '2');
    assert(rb[2] == '3');

    uint8_t data;
    assert(rb.Get(data) == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 2);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == false);
    assert(rb[0] == '2');
    assert(rb[1] == '3');

    assert(rb.Get(data) == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 1);
    assert(rb.IsEmpty() == false);
    assert(rb.IsFull() == false);
    assert(rb[0] == '3');

    assert(rb.Get(data) == true);
    assert(rb.capacity() == 3);
    assert(rb.size() == 0);
    assert(rb.IsEmpty() == true);
    assert(rb.IsFull() == false);

    assert(rb.Get(data) == false);
    assert(rb.capacity() == 3);
    assert(rb.size() == 0);
    assert(rb.IsEmpty() == true);
    assert(rb.IsFull() == false);


    return 0;
}
