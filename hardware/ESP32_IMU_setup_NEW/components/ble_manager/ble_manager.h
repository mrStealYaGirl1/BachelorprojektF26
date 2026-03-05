#define SAMPLES_PER_PACKET 10

typedef struct
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    uint32_t ts_ms;

} imu_sample_pkt_t;

typedef struct
{
    imu_sample_pkt_t sample[SAMPLES_PER_PACKET];

    uint16_t seq;
    uint16_t event_id;

} ble_imu_pkt_t;