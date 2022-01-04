//
// Created by 16182 on 1/2/2022.
//

#ifndef SILSIM_SPARKFUN_U_BLOX_GNSS_ARDUINO_LIBRARY_H
#define SILSIM_SPARKFUN_U_BLOX_GNSS_ARDUINO_LIBRARY_H

#ifndef defaultMaxWait
#define defaultMaxWait 1100
#endif
#include<cstdint>

struct SFE_UBLOX_GNSS {
    bool getPVT(uint16_t maxWait = defaultMaxWait);
    int32_t getLatitude(uint16_t maxWait = defaultMaxWait);
    int32_t getLongitude(uint16_t maxWait = defaultMaxWait);
    int32_t getAltitude(uint16_t maxWait = defaultMaxWait);
    uint8_t getFixType(uint16_t maxWait = defaultMaxWait);
    uint8_t getSIV(uint16_t maxWait = defaultMaxWait);

};

#endif  // SILSIM_SPARKFUN_U_BLOX_GNSS_ARDUINO_LIBRARY_H
