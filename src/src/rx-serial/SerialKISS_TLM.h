#pragma once

#if defined(TARGET_RX)

#include "SerialIO.h"
#include "device.h"
#include "common.h"

#define PACKED __attribute__((packed))

// CRSF telemetry packets will be sent if min rate timer in [ms] has expired or packet value has changed.
#define KISS_MIN_UPDATE_RATE 5000

// Max buffer size for serial in data
#define KISS_MAX_BUF_LEN 10

#define KISS_TEMP_INDEX 0
#define KISS_VOLT_H_INDEX 1
#define KISS_VOLT_L_INDEX 2
#define KISS_CURR_H_INDEX 3
#define KISS_CURR_L_INDEX 4
#define KISS_MAH_H_INDEX 5
#define KISS_MAH_L_INDEX 6
#define KISS_RPM_H_INDEX 7
#define KISS_RPM_L_INDEX 8
#define KISS_CRC_INDEX 9

//
// ESC data frame data structure
//
typedef struct KISS_ESC_MSG_s
{
    int8_t temperature = 0;     // 0 ESC temperature
    uint16_t voltage = 0;       // 1 Voltage in 0.01 steps
    uint16_t current = 0;       // 3 Current in 0.01 steps
    uint16_t mAhConsumed = 0;   // 5 mAhConsumed
    uint16_t rpm = 0;           // 7 RPM in 10U/min steps
    uint8_t crc = 0;            // 9 CRC
} PACKED ESCPacket_t;

class SerialKISS_TLM final : public SerialIO
{
public:
    SerialKISS_TLM(Stream &out, Stream &in, int8_t serial1TXpin = UNDEF_PIN);
    ~SerialKISS_TLM() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override { return DURATION_IMMEDIATELY; };

    int getMaxSerialReadSize() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

private:
    void processBytes(uint8_t *bytes, u_int16_t size) override;

    uint32_t htobe24(uint32_t val);

    // last received KISS telemetry packet
    ESCPacket_t esc;
    
    bool telemetryReceived = false;
    uint32_t lastESCCRC = 0;
    uint32_t lastTelemSent = 0;
};

#endif
