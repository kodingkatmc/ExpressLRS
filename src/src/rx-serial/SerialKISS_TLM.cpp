#if defined(TARGET_RX)

#include "SerialKISS_TLM.h"

#include "CRSFRouter.h"
#include "common.h"

// #define DEBUG_TEST_KISS

SerialKISS_TLM::SerialKISS_TLM(Stream &out, Stream &in, const int8_t serial1TXpin)
    : SerialIO(&out, &in) { }

int SerialKISS_TLM::getMaxSerialReadSize()
{
    return KISS_MAX_BUF_LEN;
}

void SerialKISS_TLM::sendQueuedData(uint32_t maxBytesToSend)
{

#if defined(DEBUG_TEST_KISS)
    esc.temperature = 25; // 25 C
    esc.voltage = 2521; // 25.21 V
    esc.current = 1234; // 12.34 A
    esc.mAhConsumed = 4561; // 4561 mAh
    esc.rpm = 93; // 9300 ERPM -> 930 RPM (20 pole motor)
    esc.crc += 1;
    telemetryReceived = true;
#endif

    uint32_t now = millis();
    
    // Send packet only if
    // - The receiver is connected
    // - A KISS frame was received
    // - The KISS_MIN_UPDATE_RATE has been reached or esc.CRC has changed (proxy for data is different from previously sent)
    //      TODO: There are rare cases where the esc.CRC may not change but new data has been receieved. <0.4%
    if (connectionState != connected || !telemetryReceived || (lastESCCRC == esc.crc && now - lastTelemSent <= KISS_MIN_UPDATE_RATE)) { return; }

    lastTelemSent = now;
    telemetryReceived = false;

    // indicate external sensor is present
    crsfBatterySensorDetected = true;
    
    // Prepare battery CRSF telemetry packet
    CRSF_MK_FRAME_T(crsf_sensor_battery_t) crsfBatt = {0};
    crsfBatt.p.voltage = htobe16(esc.voltage); // KISS 1=0.01V | CRSF 1=0.1V | Don't multiply by 10 to get full res on
    crsfBatt.p.current = htobe16(esc.current); // KISS 1=0.01A | CRSF 1=0.1A | transmitter. (Transmitter divides by 10)
    crsfBatt.p.capacity = htobe24(esc.mAhConsumed); // Using this for consumed capacity
    // KISS does not provide battery % telemetry
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfBatt, CRSF_FRAMETYPE_BATTERY_SENSOR, CRSF_FRAME_SIZE(2 + 3 + 3)); // sizeof(crsfBatt.p.voltage) + sizeof(crsfBatt.p.current) + sizeof(crsfBatt.p.capacity)
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfBatt.h);

    // Prepare RPM CRSF telemetry packet
    CRSF_MK_FRAME_T(crsf_sensor_rpm_t) crsfRPM = {0};
    crsfRPM.p.source_id = 0;
    crsfRPM.p.rpm0 = htobe16(esc.rpm * 100); // KISS 1=100 ERPM | CRSF=1=1ERPM | (2*ERPM/Poles = RPM)
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfRPM, CRSF_FRAMETYPE_RPM, CRSF_FRAME_SIZE(1 + 3)); // sizeof(crsfRPM.p.source_id) + sizeof(crsfRPM.p.rpm0)
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfRPM.h);

    // Prepare Temp CRSF telemetry packet
    CRSF_MK_FRAME_T(crsf_sensor_temp_t) crsfTemp = {0};
    crsfTemp.p.source_id = 0;
    crsfTemp.p.temperature[0] = htobe16(esc.temperature * 10); // KISS 1=1c | CRSF 1=0.1C
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfTemp, CRSF_FRAMETYPE_TEMP, CRSF_FRAME_SIZE(sizeof(crsfTemp.p.source_id) + sizeof(crsfTemp.p.temperature[0])));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfTemp.h);
}

void SerialKISS_TLM::processBytes(uint8_t *bytes, u_int16_t size)
{
    // TODO: maybe ensure crc checks out
    if (size == KISS_MAX_BUF_LEN && bytes[KISS_CRC_INDEX] != 0)
    {
        esc.temperature = bytes[KISS_TEMP_INDEX];
        esc.voltage = (bytes[KISS_VOLT_H_INDEX] << 8) | bytes[KISS_VOLT_L_INDEX];
        esc.current = (bytes[KISS_CURR_H_INDEX] << 8) | bytes[KISS_CURR_L_INDEX];
        esc.mAhConsumed = (bytes[KISS_MAH_H_INDEX] << 8) | bytes[KISS_MAH_L_INDEX];
        esc.rpm = (bytes[KISS_RPM_H_INDEX] << 8) | bytes[KISS_RPM_L_INDEX];
        esc.crc = bytes[KISS_CRC_INDEX];

        telemetryReceived = true;
    }
}

uint32_t SerialKISS_TLM::htobe24(uint32_t val)
{
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    return val;
#else
    uint8_t *ptrByte = (uint8_t *)&val;

    uint8_t swp = ptrByte[0];
    ptrByte[0] = ptrByte[2];
    ptrByte[2] = swp;

    return val;
#endif
}

#endif
