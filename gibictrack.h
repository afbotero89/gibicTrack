#ifndef GIBICTRACK_H
#define GIBICTRACK_H

#include "gibicTrack_global.h"
#include "RaabAlgorithm.h"

#include <QtSerialPort/QSerialPort>

#include "igtl/igtlTransformMessage.h"
#include "igtl/igtlPositionMessage.h"
#include "igtl/igtlClientSocket.h"

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

    void print_vector(const char *title, complex *x, int n);

    void Conectar3DSlicer();

    void EmpaquetarDatos(const uchar *datos);

    void EnviarPosicion(float *PosVec, float *QtrnVec);

private slots:

    void readData();

    void handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort *serial;
};

#endif // GIBICTRACK_H
