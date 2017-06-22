#ifndef DATA_MAN_H
#define DATA_MAN_H

#include <QCoreApplication>
#include <stdint.h>

#define ST_GET_START    0
#define ST_GET_TYP      1
#define ST_GET_LEN_L    2
#define ST_GET_LEN_H    3
#define ST_GET_DATA     4
#define ST_GET_CHK_L    5
#define ST_GET_CHK_M1   6
#define ST_GET_CHK_M2   7
#define ST_GET_CHK_H    8
#define ST_ESC_NEXT     9
#define ST_STAND_BY     10

#define DATA_ADC_X      0
#define DATA_ADC_Y      1
#define DATA_ADC_Z      2
#define DATA_IMU_AGM    3
#define DATA_QUAT       4

#define IMU_DATA_LEN    9
#define QUAT_DATA_LEN   4

#define PROT_START      0x7E
#define PROT_MARK       0x7D
#define PROT_MASK       0x20

extern QByteArray qbar;

class data_man
{
private:
    unsigned char fr_start;
    unsigned char fr_type;
    uint16_t fr_datax_len;
    uint16_t fr_datay_len;
    uint16_t fr_dataz_len;
    uint16_t fr_raw_len;
    uint8_t * fr_data;
    uint8_t fr_checksum;
    uint8_t my_checksum;
    unsigned char fr_status;
    bool fr_available;
    bool adc_xyz_complete;
    bool imu_data_available;
    bool quat_data_available;
public:
    data_man();
    ~data_man();
    unsigned char get_start();
    unsigned char get_type();
    int get_adcx_len();
    int get_adcy_len();
    int get_adcz_len();
    int get_raw_len();
    uint8_t get_checksum();
    uint8_t compute_checksum();
    bool data_integrity();
    void decode_frame_1(QByteArray qb);
    void decode_frame_2(QByteArray qb);
    void process_frame();
    void print_data();
    bool available();
    bool task_adc_complete();
    bool task_imu_complete();
    bool task_quat_complete();

    float * imu_data;
    float * quat_data;
    int16_t * adc_x_data;
    int16_t * adc_y_data;
    int16_t * adc_z_data;
};

#endif // DATA_MAN_H
