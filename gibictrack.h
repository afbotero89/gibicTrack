#ifndef GIBICTRACK_H
#define GIBICTRACK_H

#include "gibicTrack_global.h"
#include "RaabAlgorithm.h"

#include <QtSerialPort/QSerialPort>

typedef float real;
typedef struct{real Re; real Im;} complex;

class GIBICTRACKSHARED_EXPORT GibicTrack: QObject {

    Q_OBJECT

public:
    GibicTrack();

    bool ConectarSensor();

    void closeSerialPort();

    void SolicitarDato();

    void initActionsConnections();

    void OrganizarDatos(const uchar *datos);

    void RealizarFFTs();

    void RealizarFFT(double *signalRx);

    void fft( complex *v, int n, complex *tmp );

    void getMagnitudeVector(complex *v, double binsMatriz[3][3], double *magnitudeVector, double *frecuencyVector, int m);

    void print_vector(const char *title, complex *x, int n);

private slots:

    void readData();

    void handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort *serial;
};

#endif // GIBICTRACK_H
