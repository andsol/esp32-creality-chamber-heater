// Native tests: compile this file with a C++11 or newer compiler.
#include "../include/ThermalControl.h"
#include <cassert>
#include <cstdio>
using namespace thermal;
Input normal() {
    Input x; x.bottom = 25; x.top = 26; x.radiator = 25;
    x.bottomValid = x.topValid = x.radiatorValid = true; x.target = 60;
    return x;
}
Controller configured() {
    Controller c; c.config.soft = 90; c.config.hard = 110; c.config.minFan = 35;
    assert(c.config.valid()); return c;
}
void run(Controller& c, Input& x, uint32_t& now, unsigned seconds) {
    for (unsigned i=0; i<seconds*5; ++i) { now += 200; c.tick(x,now); }
}
int main() {
    // Test values describe a synthetic plant, not recommended hardware limits.
    auto x = normal(); uint32_t now = 0;
    Controller unconfigured; unconfigured.tick(x,now);
    assert(unconfigured.heater == 0 && unconfigured.state == State::Unconfigured);
    auto c = configured(); c.tick(x,now);
    assert(c.state == State::Spinup && c.heater == 0 && c.fan == 100);
    run(c,x,now,1); assert(c.heater == 0);
    run(c,x,now,1); assert(c.state == State::Heating && c.heater > 0);
    assert(c.fan >= c.config.minFan);
    x.radiator = 100; run(c,x,now,1);
    assert(c.heater <= 43.00001 && c.fan == 100);
    x.radiator = 110; run(c,x,now,1);
    assert(c.state == State::Fault && c.heater == 0 && c.fan == 100);
    x.radiator = 25; run(c,x,now,1);
    assert(c.state == State::Fault && !c.reset(x)); // No automatic restart.
    x.target = 0; assert(!c.reset(x)); run(c,x,now,30); assert(c.reset(x)); assert(c.heater == 0 && c.state == State::Idle);
    x = normal(); c.tick(x,now); run(c,x,now,3);
    x.radiatorValid = false; c.tick(x,now+200);
    assert(c.state == State::Fault && c.heater == 0 && c.fan == 100);
    assert(!c.reset(x));
    c = configured(); x = normal(); now = 0; c.tick(x,now); run(c,x,now,3);
    x.target = 0; x.radiator = 60; run(c,x,now,1);
    assert(c.state == State::Cooldown && c.heater == 0 && c.fan >= 35);
    x.radiator = 29; run(c,x,now,29); assert(c.state == State::Cooldown);
    run(c,x,now,2); assert(c.state == State::Idle && c.fan == 0);
    c = configured(); x = normal(); x.useBottom=false; x.topValid=false; c.tick(x,0);
    assert(c.state == State::Fault && c.heater == 0);
    c = configured(); x = normal(); x.topValid=false; c.tick(x,0);
    assert(c.state == State::Spinup); // Local source does not require Moonraker.
    c = configured(); x = normal(); x.bottom=70; c.tick(x,0);
    assert(c.state == State::Fault && c.heater == 0);
    c = configured(); x = normal(); c.tick(x,0); c.tick(x,3000);
    assert(c.state == State::Fault); // Late control update.
    c = configured(); x = normal(); now=0xfffffc00; c.tick(x,now); run(c,x,now,3);
    assert(c.state == State::Heating); // millis rollover.
    c = configured(); c.config.verifySeconds=60; x = normal(); now=0;
    c.tick(x,now); run(c,x,now,65); assert(c.state==State::Fault && c.heater==0);
    c = configured(); c.config.verifySeconds=60; x = normal(); now=0; c.tick(x,now);
    run(c,x,now,40); x.bottom+=1.1; run(c,x,now,40); assert(c.state==State::Heating);
    // Saturated PI must not store a large integral while radiator limits power.
    c=configured(); c.config.verifySeconds=3600; x=normal(); x.radiator=109; now=0;
    c.tick(x,now); run(c,x,now,300); assert(c.heater<=4.30001);
    x.radiator=25; x.bottom=60; run(c,x,now,1); assert(c.heater<0.01);
    c=configured(); c.config.minFan=NAN; assert(!c.config.valid());
    c=configured(); c.config.hard=140; assert(!c.config.valid());
    c=configured(); x=normal(); x.target=NAN; c.tick(x,0); assert(c.state==State::Fault);
    // A hot boot with heat off still starts the cooling fan at full PWM.
    c=configured(); x=normal(); x.target=0; x.radiator=60; c.tick(x,0);
    assert(c.state==State::Cooldown && c.heater==0 && c.fan==100);
    // The optional top sensor still trips overtemperature when available.
    c=configured(); x=normal(); x.top=70; c.tick(x,0); assert(c.state==State::Fault);
    // Local sensor is required even with the remote temperature selected.
    c=configured(); x=normal(); x.useBottom=false; x.bottomValid=false;
    c.tick(x,0); assert(c.state==State::Fault);
    // Setup-required cooling must not switch off between noisy NTC samples.
    {
        Controller setup;
        auto sample = normal(); sample.target = 0;
        uint32_t clock = 0;
        setup.tick(sample, clock);
        assert(setup.fan == 100 && setup.heater == 0);
        run(setup, sample, clock, 29); assert(setup.fan == 100);
        run(setup, sample, clock, 2); assert(setup.fan == 0);
        for (int i = 0; i < 40; ++i) {
            sample.radiator = sample.bottom + 6;
            run(setup, sample, clock, 1); assert(setup.fan == 100);
            sample.radiator = sample.bottom + 4;
            run(setup, sample, clock, 1);
            assert(setup.fan == 100 && setup.heater == 0);
        }
        run(setup, sample, clock, 28); assert(setup.fan == 100);
        sample.radiatorValid = false;
        run(setup, sample, clock, 1); assert(setup.fan == 100);
        sample.radiatorValid = true;
        run(setup, sample, clock, 29); assert(setup.fan == 100);
        run(setup, sample, clock, 2); assert(setup.fan == 0);
        sample.bottomValid = false;
        run(setup, sample, clock, 1); assert(setup.fan == 100);
        sample.bottomValid = true;
        run(setup, sample, clock, 31); assert(setup.fan == 0);
        assert(setup.state == State::Unconfigured && setup.heater == 0);
    }
    // A bounded sweep catches non-finite outputs and violations of hard limits.
    for (int temp=20;temp<=120;temp+=5) {
        for (int target=0;target<=65;target+=5) {
            c=configured(); x=normal(); x.radiator=temp; x.target=target; now=0;
            c.tick(x,now); run(c,x,now,4);
            assert(std::isfinite(c.heater) && std::isfinite(c.fan));
            assert(c.heater>=0 && c.heater<=86 && c.fan>=0 && c.fan<=100);
            if (target==0 || temp>=110) assert(c.heater==0);
            if (c.heater>0) assert(c.fan>=c.config.minFan);
        }
    }
    std::puts("Thermal control scenarios and output-bound sweep passed");
}
