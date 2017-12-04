#include "RingIndex.h"

//#include <iostream>
//#include <string>
#include <assert.h>




int main() {
    TRingIndex ri {3};

    assert(ri.size() == 3);
    assert(ri.idx == 0);
    assert(ri.CalcFwd() == 1);
    assert(ri.CalcFwd(2) == 2);
    assert(ri.CalcFwd(3) == 0);
    assert(ri.CalcFwd(7) == 1);
    ri.Fwd();
    assert(ri.size() == 3);
    assert(ri.idx == 1);
    assert(ri.CalcFwd() == 2);
    assert(ri.CalcFwd(2) == 0);
    assert(ri.CalcFwd(3) == 1);
    assert(ri.CalcFwd(7) == 2);
    ri.Fwd();
    assert(ri.size() == 3);
    assert(ri.idx == 2);
    assert(ri.CalcFwd() == 0);
    assert(ri.CalcFwd(2) == 1);
    assert(ri.CalcFwd(3) == 2);
    assert(ri.CalcFwd(7) == 0);
    ri.Fwd();
    assert(ri.size() == 3);
    assert(ri.idx == 0);
    assert(ri.CalcFwd() == 1);
    assert(ri.CalcFwd(2) == 2);
    assert(ri.CalcFwd(3) == 0);
    assert(ri.CalcFwd(7) == 1);

    return 0;
}
