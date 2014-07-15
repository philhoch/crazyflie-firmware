#include "sensordata.h"
#include "crtp.h"

#define PACKET1_CH      0
#define PACKET2_CH      1

static CRTPPacket pk1;
static CRTPPacket pk2;

void sendSensorData(uint64_t* ts, float* roll, float *pitch, float* yaw, float* acc_x, float* acc_y, float* acc_z, float* gyro_x, float* gyro_y, float* gyro_z) {
    pk1.header = CRTP_HEADER(CRTP_PORT_SENSORDATA, PACKET1_CH);
    pk1.size=28;
    pk1.data[0] = *ts;
    pk1.data[8] = *roll;
    pk1.data[12] = *pitch;
    pk1.data[16] = *yaw;
    pk1.data[20] = *acc_x;
    pk1.data[24] = *acc_y;
    crtpSendPacket(&pk1);

    pk2.header = CRTP_HEADER(CRTP_PORT_SENSORDATA, PACKET2_CH);
    pk2.size=24;
    pk2.data[0] = *ts;
    pk2.data[8] = *acc_z;
    pk2.data[12] = *gyro_x;
    pk2.data[16] = *gyro_y;
    pk2.data[20] = *gyro_z;
    crtpSendPacket(&pk2);
}

