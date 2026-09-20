#include "../include/NtcProcessing.h"
#include <cassert>
#include <cstdio>

int main() {
    ntc::Filter filter;
    assert(filter.update(32) == 32);
    for (int i = 0; i < 120; ++i) {
        const float value = filter.update(i % 2 ? 31 : 36);
        if (i > 30) assert(value > 33 && value < 34);
    }
    assert(std::isnan(filter.update(NAN)));
    assert(filter.update(90) == 90); // No old cool value after a missing sample.
    filter.reset();
    assert(filter.update(25) == 25);
    for (int i = 0; i < 30; ++i) filter.update(60);
    assert(filter.update(60) > 59.9f); // Sustained changes are followed.

    ntc::ReferenceWindow reference;
    for (int i = 0; i < 59; ++i) reference.add(33, 29.7f, true);
    assert(!reference.ready() && std::isnan(reference.offset()));
    reference.add(33, 29.7f, true);
    assert(reference.ready() && std::fabs(reference.offset() + 3.3f) < 0.001f);
    reference.add(36.06f, 29.7f, true);
    assert(reference.ready()); // The observed 3.06 C noise is accepted.
    reference.add(37, 29.7f, true);
    assert(reference.ready()); // Inclusive 4 C limit.
    reference.add(37.01f, 29.7f, true);
    assert(!reference.ready());
    for (int i = 0; i < 60; ++i) reference.add(33, 29.7f, true);
    assert(reference.ready());
    reference.add(33, 31, true); assert(!reference.ready());
    reference.add(NAN, 29.7f, true); assert(!reference.ready());
    for (int i = 0; i < 60; ++i) reference.add(33, 29.7f, true);
    reference.add(33, 29.7f, false); assert(!reference.ready());
    reference.reset(); assert(!reference.ready());
    std::puts("NTC filtering and calibration scenarios passed");
}
