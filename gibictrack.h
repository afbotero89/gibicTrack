#ifndef GIBICTRACK_H
#define GIBICTRACK_H

#include "gibicTrack_global.h"

#include <QtSerialPort/QSerialPort>

typedef float real;
typedef struct{real Re; real Im;} complex;

class GIBICTRACKSHARED_EXPORT GibicTrack: QObject {

public:
    GibicTrack();

    bool ConectarSensor();

    void closeSerialPort();

    void SolicitarDato();

    void initActionsConnections();

    void OrganizarDatos(const uchar *datos);

    void fft( complex *v, int n, complex *tmp );

private slots:

    void readData();

    void handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort *serial;
};

#endif // GIBICTRACK_H
