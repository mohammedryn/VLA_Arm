// firmware/src/sensor_safety/test_sensor_comms.cpp
#include <cassert>
#include <cstdio>
#include "sensor_comms.h"

int main() {
    assert(sizeof(SensorStatus_t) == 168);

    SensorStatus_t pkt = {};
    pkt.magic = SENSOR_PACKET_MAGIC;
    pkt.contact_flag = 1;
    pkt.estop_active = 0;
    pkt.tof_mm[0] = 0xFFFF;
    pkt.tof_mm[63] = 1234;

    assert(pkt.magic == 0xBEEFCAFEUL);
    assert(pkt.contact_flag == 1);
    assert(pkt.tof_mm[0] == 0xFFFF);
    assert(pkt.tof_mm[63] == 1234);

    printf("PASS: SensorStatus_t is %zu bytes\n", sizeof(SensorStatus_t));
    return 0;
}
