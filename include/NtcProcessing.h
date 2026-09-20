#pragma once
#include <cmath>
#include <algorithm>

namespace ntc {
// At 1 Hz, alpha=0.2 attenuates sample-to-sample noise; time constant ~4.5 s.
// Invalid samples propagate immediately rather than retaining a plausible value.
class Filter {
    float value = NAN;
public:
    void reset() { value = NAN; }
    float update(float sample) {
        if (!std::isfinite(sample)) { reset(); return NAN; }
        value = std::isfinite(value) ? value + 0.2f * (sample - value) : sample;
        return value;
    }
};

// Rolling one-minute reference using uncorrected filtered NTC and local DHT20.
class ReferenceWindow {
    float radiator[60] = {}, air[60] = {};
    unsigned count = 0, next = 0;
public:
    unsigned sampleCount() const { return count; }
    void reset() { count = next = 0; }
    void add(float n, float d, bool heatOff) {
        if (!heatOff || !std::isfinite(n) || !std::isfinite(d)) { reset(); return; }
        radiator[next] = n; air[next] = d;
        next = (next + 1) % 60;
        if (count < 60) ++count;
    }
    float ntcSpread() const {
        if (!count) return NAN;
        const auto limits = std::minmax_element(radiator, radiator + count);
        return *limits.second - *limits.first;
    }
    float dhtSpread() const {
        if (!count) return NAN;
        const auto limits = std::minmax_element(air, air + count);
        return *limits.second - *limits.first;
    }
    bool ready() const {
        if (count < 60) return false;
        const auto n = std::minmax_element(radiator, radiator + 60);
        const auto d = std::minmax_element(air, air + 60);
        return *n.second - *n.first <= 4.0f && *d.second - *d.first <= 0.5f;
    }
    float offset() const {
        if (!ready()) return NAN;
        float sum = 0;
        for (unsigned i = 0; i < 60; ++i) sum += air[i] - radiator[i];
        return sum / 60;
    }
};
} // namespace ntc
