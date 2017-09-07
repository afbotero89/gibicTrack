#ifndef GIBICTRACK_H
#define GIBICTRACK_H

#include "gibicTrack_global.h"
#include "RaabAlgorithm.h"

#include <QtSerialPort/QSerialPort>

#include "igtl/igtlTransformMessage.h"
#include "igtl/igtlPositionMessage.h"
#include "igtl/igtlClientSocket.h"
#include "data_man.h"
#include <QTimer>

#define q    9        /* for 2^7 points --- Señal de 2^n datos */
#define N    (1<<q)        /* N-point FFT, iFFT */

typedef float real;
typedef struct{real Re; real Im;} complex;

class GIBICTRACKSHARED_EXPORT GibicTrack: QObject {

    Q_OBJECT

public:
    GibicTrack();

    bool ConectarSensor();

    void closeSerialPort();

    void SolicitarDato(double retorno[3][2]);

    void initActionsConnections();

    void OrganizarDatos(const uchar *datos);

    void RealizarFFTs();

    void RealizarFFT(double *signalRx);

    void fft( complex *v, int n, complex *tmp );

    void getMagnitudeVector(complex *v, double binsMatriz[3][3], double *magnitudeVector, double *frecuencyVector, int m);

    void Conectar3DSlicer();

    void EmpaquetarDatos(const uchar *datos);

    void EnviarPosicion(float *PosVec, float *QtrnVec);

    void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

    void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    float invSqrt(float x);

    void SolicitarDatoIMUTimer();

    void binaryToFloat(float *array, QByteArray Qdata);

    void readDataIMU(int total, QByteArray Qdata);

    void solicitaSenalesTimer(double magVec_graph[128/2], double frecVec_graph[128/2]);

    void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    void readQuaternions(QByteArray Qdata);

private slots:

    void readData();

    void handleError(QSerialPort::SerialPortError error);

    void solicitaDatoIMU();

    void solicitaSenales();

private:
    QSerialPort *serial;
    data_man *dm;
    int totalDatos;
    bool SlicerConectado;
    bool GibicConectado;
    bool SoketCreado;
    int total;

};

#endif // GIBICTRACK_H
