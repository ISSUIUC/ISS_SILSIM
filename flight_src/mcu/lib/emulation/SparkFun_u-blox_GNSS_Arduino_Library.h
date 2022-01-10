//
// Created by 16182 on 1/2/2022.
//

#ifndef SILSIM_SPARKFUN_U_BLOX_GNSS_ARDUINO_LIBRARY_H
#define SILSIM_SPARKFUN_U_BLOX_GNSS_ARDUINO_LIBRARY_H

#ifndef defaultMaxWait
#define defaultMaxWait 1100
#endif
#include<cstdint>
#include "SPI.h"

#define COM_PORT_SPI 4
#define COM_TYPE_UBX (1 << 0)

#define VAL_CFG_SUBSEC_IOPORT 0x00000001

struct SFE_UBLOX_GNSS {
    bool getPVT(uint16_t maxWait = defaultMaxWait);
    int32_t getLatitude(uint16_t maxWait = defaultMaxWait);
    int32_t getLongitude(uint16_t maxWait = defaultMaxWait);
    int32_t getAltitude(uint16_t maxWait = defaultMaxWait);
    uint8_t getFixType(uint16_t maxWait = defaultMaxWait);
    uint8_t getSIV(uint16_t maxWait = defaultMaxWait);
    bool begin(SPIClass &spiPort, uint8_t csPin, uint32_t spiSpeed);
    bool setPortOutput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = (uint16_t)1100U);
    bool saveConfigSelective(uint32_t configMask, uint16_t maxWait = (uint16_t)1100U);
    bool setNavigationFrequency(uint8_t navFreq, uint16_t maxWait = (uint16_t)1100U);
};

#endif  // SILSIM_SPARKFUN_U_BLOX_GNSS_ARDUINO_LIBRARY_H
