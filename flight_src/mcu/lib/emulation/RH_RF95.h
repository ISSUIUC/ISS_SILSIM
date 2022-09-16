
#ifndef RH_RF95_H
#define RH_RF95_H
class RH_RF95 {
   public:
    RH_RF95(uint8_t cs, uint8_t in);
    bool init();
    bool setFrequency(float centre);
    void setTxPower(uint8_t power, bool useRFO = false);
    bool send(uint8_t* data, uint8_t len);
    uint8_t lastRssi();
    void waitPacketSent();
};

#endif