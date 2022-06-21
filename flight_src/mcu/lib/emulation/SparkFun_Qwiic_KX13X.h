#include<SPI.h>
#include<cstdint>

struct outputData {
    float xData;
    float yData;
    float zData;
};

struct rawOutputData {
    float xData;
    float yData;
    float zData;
};

class QwiicKX134 {
    public:
        QwiicKX134() = default;
        bool beginSPI(uint8_t, uint32_t, SPIClass &spiPort = SPI){ return true; }
        outputData getAccelData();
};