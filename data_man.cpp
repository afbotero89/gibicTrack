#include "data_man.h"
#include <QDebug>

data_man::data_man()
{

    //this->imu_data = new float[IMU_DATA_LEN];
    //this->quat_data = new float[QUAT_DATA_LEN];
    this->imu_data = NULL;
    this->quat_data = NULL;
    this->adc_x_data = NULL;
    this->adc_y_data = NULL;
    this->adc_z_data = NULL;
    this->fr_available = false;
    this->fr_status = ST_STAND_BY;
    this->adc_xyz_complete = false;
    this->imu_data_available = false;
    this->quat_data_available = false;
    this->fr_data = NULL;
    this->my_checksum = 0;
}

data_man::~data_man()
{
    //delete[] this->imu_data;
    //delete[] this->quat_data;
    delete[] this->adc_x_data;
    delete[] this->adc_y_data;
    delete[] this->adc_z_data;
    delete[] this->fr_data;
}

void data_man::decode_frame_1(QByteArray qb)
{
    static int j = 0, calc = 0;
    int i;
    unsigned char data;
    for(i = 0; i < qb.length(); i++)
    {
        data = qb.at(i);
        switch(this->fr_status)
        {
            case ST_GET_START:
                if(data == 0x7E)
                {
                    //qDebug() << "<START>" << (void *)data;
                    this->fr_start = data;
                    this->fr_status = ST_GET_TYP;
                    this->fr_available = false;
                }
                break;
            case ST_GET_TYP:
                //qDebug() << "<TYPE>" << (void *)data;
                this->fr_type = data;
                if(data == DATA_ADC_X || data == DATA_ADC_Y || data == DATA_ADC_Z)
                {
                    this->adc_xyz_complete = false;
                    this->fr_status = ST_GET_LEN_L;
                }
                else if(data == DATA_IMU_AGM)
                {
                    this->fr_raw_len = 4*IMU_DATA_LEN;
                    delete [] this->fr_data;
                    this->fr_data = new uint8_t[4*IMU_DATA_LEN];
                    this->fr_status = ST_GET_DATA;
                    this->imu_data_available = false;
                    this->my_checksum = 0;
                    j = 0;
                }
                else if(data == DATA_QUAT)
                {
                    this->fr_raw_len = 4*QUAT_DATA_LEN;
                    delete [] this->fr_data;
                    this->fr_data = new uint8_t[4*QUAT_DATA_LEN];
                    this->fr_status = ST_GET_DATA;
                    this->quat_data_available = false;
                    this->my_checksum = 0;
                    j = 0;
                }
                break;
            case ST_GET_LEN_L:
                calc = data & 0xFF;
                this->fr_status = ST_GET_LEN_H;
                break;
            case ST_GET_LEN_H:
                calc |= (data & 0xFF) << 8;
                this->fr_raw_len = calc;
                this->fr_status = ST_GET_DATA;
                delete [] this->fr_data;
                this->fr_data = new uint8_t[this->get_raw_len()];
                this->my_checksum = 0;
                j = 0;
                break;
            case ST_GET_DATA:
                this->fr_data[j++] = data;
                this->my_checksum ^= data;
                if(j == this->get_raw_len())
                    this->fr_status = ST_GET_CHK_L;
                break;
            case ST_GET_CHK_L:
                this->fr_checksum = data;
                this->fr_available = true;
                this->process_frame();
                //this->print_data();
                this->fr_status = ST_GET_START;
                //qDebug() << "<<<END>>>";
                break;
            default:
                this->fr_status = ST_GET_START;
                break;
        }
        //qDebug() << (void *)data;
    }
}

void data_man::decode_frame_2(QByteArray qb)
{
    static int j = 0;
    static uint16_t pkg_len = 0;
    static bool esc_next = false;
    int i;
    unsigned char data;
    for(i = 0; i < qb.length(); i++)
    {
        data = qb.at(i);
        switch(data)
        {
            case PROT_START:
                //qDebug() << "<START>" << (void *)data;
                this->fr_status = ST_GET_TYP;
                break;
            case PROT_MARK:
                esc_next = true;
                break;
            default:
                if(esc_next)
                {
                    data ^= PROT_MASK;
                    esc_next = false;
                }
                switch(this->fr_status)
                {
                    case ST_GET_TYP:
                        //qDebug() << "<TYPE>" << (void *)data;
                        this->fr_type = data;
                        if(data == DATA_ADC_X || data == DATA_ADC_Y || data == DATA_ADC_Z)
                        {
                            //this->adc_xyz_complete = false;
                            this->fr_status = ST_GET_LEN_L;
                        }
                        else if(data == DATA_IMU_AGM)
                        {
                            this->fr_raw_len = 4*IMU_DATA_LEN;
                            delete [] this->fr_data;
                            this->fr_data = new uint8_t[4*IMU_DATA_LEN];
                            this->fr_status = ST_GET_DATA;
                            this->my_checksum = 0;
                            j = 0;
                        }
                        else if(data == DATA_QUAT)
                        {
                            this->fr_raw_len = 4*QUAT_DATA_LEN;
                            delete [] this->fr_data;
                            this->fr_data = new uint8_t[4*QUAT_DATA_LEN];
                            this->fr_status = ST_GET_DATA;
                            this->my_checksum = 0;
                            j = 0;
                        }
                        break;
                    case ST_GET_LEN_L:
                        //qDebug() << "<LEN_L>" << (void *)data;
                        pkg_len = data & 0xFF;
                        this->fr_status = ST_GET_LEN_H;
                        break;
                    case ST_GET_LEN_H:
                        //qDebug() << "<LEN_H>" << (void *)data;
                        pkg_len |= (data & 0xFF) << 8;
                        this->fr_raw_len = pkg_len;
                        this->fr_status = ST_GET_DATA;
                        delete [] this->fr_data;
                        this->fr_data = new uint8_t[pkg_len];
                        this->my_checksum = 0;
                        j = 0;
                        break;
                    case ST_GET_DATA:
                        //qDebug() << "<DATA>" << (void *)data;
                        this->my_checksum ^= data;
                        this->fr_data[j++] = data;
                        if(j == this->get_raw_len())
                            this->fr_status = ST_GET_CHK_L;
                        break;
                    case ST_GET_CHK_L:
                        //qDebug() << "<CHECKSUM>" << (void *)data;
                        this->fr_checksum = data;
                        this->fr_available = true;
                        this->process_frame();
                        //this->print_data();
                        this->fr_status = ST_STAND_BY;
                        break;
                    case ST_STAND_BY:
                        break;
                    default:
                        this->fr_status = ST_STAND_BY;
                        break;
                }
                break;
        }

    }
}

void data_man::process_frame()
{
    int i = 0, j = 0;
    int16_t tmp16;
    uint32_t tmp32;

    if(this->data_integrity())
    {
        switch(this->get_type())
        {
            case DATA_ADC_X:
                this->fr_datax_len = this->get_raw_len()/2;
                delete [] this->adc_x_data;
                this->adc_x_data = new int16_t[this->get_adcx_len()];
                for(i = 0; i < this->get_raw_len(); i += 2)
                {
                    tmp16 = (this->fr_data[i] & 0xFF) | ((this->fr_data[i + 1] & 0xFF) << 8);
                    this->adc_x_data[j++] = tmp16;
                }
                break;
            case DATA_ADC_Y:
                this->fr_datay_len = this->get_raw_len()/2;
                delete [] this->adc_y_data;
                this->adc_y_data = new int16_t[this->get_adcy_len()];
                for(i = 0; i < this->get_raw_len(); i += 2)
                {
                    tmp16 = (this->fr_data[i] & 0xFF) | ((this->fr_data[i + 1] & 0xFF) << 8);
                    this->adc_y_data[j++] = tmp16;
                }
                break;
            case DATA_ADC_Z:
                this->fr_dataz_len = this->get_raw_len()/2;
                delete [] this->adc_z_data;
                this->adc_z_data = new int16_t[this->get_adcz_len()];
                for(i = 0; i < this->get_raw_len(); i += 2)
                {
                    tmp16 = (this->fr_data[i] & 0xFF) | ((this->fr_data[i + 1] & 0xFF) << 8);
                    this->adc_z_data[j++] = tmp16;
                }
                this->adc_xyz_complete = true;
                break;
            case DATA_IMU_AGM:
                /*for(i = 0; i < IMU_DATA_LEN*4; i += 4)
                {
                    tmp32 = this->fr_data[i] & 0xFF;
                    tmp32 |= (this->fr_data[i + 1] & 0xFF) << 8;
                    tmp32 |= (this->fr_data[i + 2] & 0xFF) << 16;
                    tmp32 |= (this->fr_data[i + 3] & 0xFF) << 24;
                    this->imu_data[j++] = *(float *)&tmp32;
                }*/
                this->imu_data = (float *)this->fr_data;
                this->imu_data_available = true;
                break;
            case DATA_QUAT:
                /*for(i = 0; i < QUAT_DATA_LEN*4; i += 4)
                {
                    tmp32 = this->fr_data[i] & 0xFF;
                    tmp32 |= (this->fr_data[i + 1] & 0xFF) << 8;
                    tmp32 |= (this->fr_data[i + 2] & 0xFF) << 16;
                    tmp32 |= (this->fr_data[i + 3] & 0xFF) << 24;
                    this->quat_data[j++] = *(float *)&tmp32;
                }*/
                this->quat_data = (float *)this->fr_data;
                this->quat_data_available = true;
                break;
        }
    }
    else
        qDebug() << "Data integrity failed!! (" << this->get_type() << "," << this->get_checksum() << "," << this->compute_checksum() << "," << this->get_raw_len() << ")";
}

void data_man::print_data()
{
    int i;
    switch(this->get_type())
    {
        case DATA_ADC_X:
            qDebug() << "ADC-x data:";
            for(i = 0; i < this->get_adcx_len(); i++)
                qDebug() << this->adc_x_data[i];
            break;
        case DATA_ADC_Y:
            qDebug() << "ADC-y data:";
            for(i = 0; i < this->get_adcy_len(); i++)
                qDebug() << this->adc_y_data[i];
            break;
        case DATA_ADC_Z:
            qDebug() << "ADC-z data:";
            for(i = 0; i < this->get_adcz_len(); i++)
                qDebug() << this->adc_z_data[i];
            break;
        case DATA_IMU_AGM:
            qDebug() << "IMU data:";
            for(i = 0; i < IMU_DATA_LEN; i++)
                qDebug() << this->imu_data[i];
            break;
        case DATA_QUAT:
            qDebug() << "Quaternions data:";
            for(i = 0; i < QUAT_DATA_LEN; i++)
                qDebug() << this->quat_data[i];
            break;
    }
}

unsigned char data_man::get_start()
{
    return this->fr_start;
}

unsigned char data_man::get_type()
{
    return this->fr_type;
}

int data_man::get_adcx_len()
{
    return this->fr_datax_len;
}

int data_man::get_adcy_len()
{
    return this->fr_datay_len;
}

int data_man::get_adcz_len()
{
    return this->fr_dataz_len;
}

int data_man::get_raw_len()
{
    return this->fr_raw_len;
}

uint8_t data_man::get_checksum()
{
    return this->fr_checksum;
}

uint8_t data_man::compute_checksum()
{
    return this->my_checksum;
}

bool data_man::data_integrity()
{
    return (this->get_checksum() == this->compute_checksum());
}

bool data_man::available()
{
    bool tmp = this->fr_available;
    this->fr_available = false;
    return tmp;
}

bool data_man::task_adc_complete()
{
    bool tmp = this->adc_xyz_complete;
    this->adc_xyz_complete = false;
    return tmp;
}

bool data_man::task_imu_complete()
{
    bool tmp = this->imu_data_available;
    this->imu_data_available = false;
    return tmp;
}

bool data_man::task_quat_complete()
{
    bool tmp = this->quat_data_available;
    this->quat_data_available = false;
    return tmp;
}
