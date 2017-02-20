#include "gibictrack.h"
#include <QtSerialPort/QSerialPort>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <QDebug>

#define q    11        /* for 2^7 points --- Señal de 2^n datos */
#define N    (1<<q)        /* N-point FFT, iFFT */

#ifndef PI
# define PI    3.14159265358979323846264338327950288
#endif

bool GibicConectado = false;
int totalDatos = 0;
int total=0;
int arregloXYZ[80000][3];//Se inicializa en el constructor de la clase (mainwindow) al reservar espacio para el maximo de bytes esperados

GibicTrack::GibicTrack()
{

    initActionsConnections();
}

void GibicTrack::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        closeSerialPort();
    }
}

bool GibicTrack::ConectarSensor()
{
    //Conexiones para las señales del puerto serial
    //connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this,SLOT(handleError(QSerialPort::SerialPortError)));
    //connect(serial, SIGNAL(readyRead()), this, SLOT(procesarDatos()));
    serial = new QSerialPort(this);
    serial->setPortName("ttyUSB0");//Se requiere saber con anterioridad el nombre asignado al puerto
    serial->setBaudRate(9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (serial->open(QIODevice::ReadWrite)) {
        // Conexion OK
        GibicConectado = true;

    } else {
        // Si hay error
        GibicConectado = false;
    }

    return GibicConectado;
}

void GibicTrack::SolicitarDato()
{
    qDebug() << "pide dato";
    totalDatos = 0;
    serial->write("$");
    readData();
}

void GibicTrack::readData()
{
    int cantidad=serial->bytesAvailable();
    totalDatos+=cantidad;
    QByteArray Qdata = serial->readAll();
    qDebug() << Qdata;
    qDebug() << totalDatos;
}

void GibicTrack::closeSerialPort()
{
    if (serial->isOpen())
        serial->close();
}

void GibicTrack::initActionsConnections()
{
    //Conexiones para las señales del puerto serial
    //connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(handleError(QSerialPort::SerialPortError)));
    //connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
}

// Funcion para organizar datos provenientes del sensor
void GibicTrack::OrganizarDatos(const uchar *datos){
    int cntPos=0;//Contador de posición en el vector, indica la columna de la matriz
    int cntVec = 0;//Contador del vector con el que se trabaja (X, Y o Z)indica la fila de la matriz

    for (int j = 0; j < total / 2; j++)//otra forma es j+=2 y que el indice sea j en lugar de 2*j
    {
        arregloXYZ[cntPos][cntVec]= datos[2 * j] * 256 + datos[2 * j + 1];
        cntPos++;
        if (cntPos % 128 == 0)
        {
            cntVec++;
            if (cntVec==3)
            {
                cntVec = 0;
            }else{
                cntPos-=128;
            }
        }
    }
}

// Funciones para calculo de FFT
void GibicTrack::RealizarFFT(double *signalRx){

    complex v[N], scratch[N];

    int k;

    /* Fill v[] with a function of known FFT: */

    //console->putData("\n orig!!");
    for(k=0; k<N; k++) {

        v[k].Re = signalRx[k];
        v[k].Im = 0;

    }

    /* FFT of v[]: */

    fft( v, N, scratch );
}


void GibicTrack::fft( complex *v, int n, complex *tmp )
{

  if(n>1) {            /* otherwise, do nothing and return */

    int k,m;    complex z, w, *vo, *ve;

    ve = tmp; vo = tmp+n/2;

    for(k=0; k<n/2; k++) {

      ve[k] = v[2*k];

      vo[k] = v[2*k+1];

    }

    fft( ve, n/2, v );        /* FFT on even-indexed elements of v[] */

    fft( vo, n/2, v );        /* FFT on odd-indexed elements of v[] */

    for(m=0; m<n/2; m++) {

      w.Re = cos(2*PI*m/(double)n);
      w.Im = -sin(2*PI*m/(double)n);
      z.Re = w.Re*vo[m].Re - w.Im*vo[m].Im;    /* Re(w*vo[m]) */
      z.Im = w.Re*vo[m].Im + w.Im*vo[m].Re;    /* Im(w*vo[m]) */
      v[  m  ].Re = ve[m].Re + z.Re;
      v[  m  ].Im = ve[m].Im + z.Im;
      v[m+n/2].Re = ve[m].Re - z.Re;
      v[m+n/2].Im = ve[m].Im - z.Im;
    }
  }
  return;
}
