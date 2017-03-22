#include "gibictrack.h"
#include <QtSerialPort/QSerialPort>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <QDebug>

#define q    7        /* for 2^7 points --- Señal de 2^n datos */
#define N    (1<<q)        /* N-point FFT, iFFT */

#ifndef PI
# define PI    3.14159265358979323846264338327950288
#endif

bool GibicConectado = false;
int totalDatos = 0;
int total=0;
double arregloXYZ[80000][3];//Se inicializa en el constructor de la clase (mainwindow) al reservar espacio para el maximo de bytes esperados
double binsMatriz[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};

const int samplingFrecuency = 25000;//80000;
const int deltaF=samplingFrecuency/(2*N);//El deltaF= fs/N pero para estrechar el intervalo en el que cae la frecuencia divido por 2, evitando coger 2 valores
//Esto hay que organizarlo las frecuencias no estan en orden
const int frecuenciesCoil[3] = {5000,10000,15000};

QByteArray vectorRX;

GibicTrack::GibicTrack()
{
    vectorRX.reserve(150000);
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
    serial->setPortName("ttyACM0");//Se requiere saber con anterioridad el nombre asignado al puerto
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
    initActionsConnections();
    return GibicConectado;
}

void GibicTrack::SolicitarDato()
{
    qDebug() << "pide dato";
    totalDatos = 0;
    serial->write("$");
}

void GibicTrack::readData()
{
    int cantidad=serial->bytesAvailable();
    total+=cantidad;
    QByteArray Qdata = serial->readAll();
    vectorRX.append(Qdata);
    double radio;
    qDebug()<< total;
    if(total>=768){//if(total>=N*3*2){
        //Se requiere de este casting para tomar los bytes del QByteArray como char sin signo
        const uchar *datosRX= reinterpret_cast<const uchar*>(vectorRX.constData());
        OrganizarDatos(datosRX);
        RealizarFFTs();
        //MostrarMatrizMag();
        //PosOri_Raab (binsMatriz, posXYZ, radio);
        //MostrarXYZ(posXYZ);

        //values[1] = posXYZ[0][0];
        //values[2] = posXYZ[1][0];
        //values[3] = 0;

        //EmpaquetarDatos(values);
    }
    //qDebug() << totalDatos;
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
    connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
}

void GibicTrack::RealizarFFTs(){
    double signalRx[N];
    complex v[N], scratch[N];

    int k;

    /* Fill v[] with a function of known FFT: */

    for(int i=0;i<3;i++){
        for(k=0; k<N; k++) {
            signalRx[k]=arregloXYZ[k][i];//sig_10_10_05[k];
            v[k].Re = arregloXYZ[k][i];
            v[k].Im = 0;

        }

        print_vector("Orig", v, N);

        /* FFT of v[]: */

        fft( v, N, scratch );

        print_vector(" FFT", v, N);

        /* Get magnitude vector */

        double magVec[N/2], frecVec[N/2];
        getMagnitudeVector(v, binsMatriz, magVec, frecVec,i);

        //Para no graficar nivel DC que al ser ondas sobre 0V siempre tienen un nivel DC alto
        magVec[0]=0;

    }
}


// Funcion para organizar datos provenientes del sensor
void GibicTrack::OrganizarDatos(const uchar *datos){
    int cntPos=0;//Contador de posición en el vector, indica la columna de la matriz
    int cntVec = 0;//Contador del vector con el que se trabaja (X, Y o Z)indica la fila de la matriz

    for (int j = 0; j < total / 2; j++)//otra forma es j+=2 y que el indice sea j en lugar de 2*j
    {
        arregloXYZ[cntPos][cntVec]= datos[2 * j] * 256 + datos[2 * j + 1];
        cntPos++;
        if (cntPos == 128)
        {
            cntVec++;
            cntPos=0;
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

// Funcion que obtiene la magnitud del vector complejo que retorna la FFT
// Calcula la matriz de bins (magnitudes para las frecuencias de 5K, 10K, 15K)
void GibicTrack::getMagnitudeVector(complex *v, double binsMatriz[3][3], double *magnitudeVector, double *frecuencyVector, int m){
    int j, k;

    for(j=0; j<N/2; j++) {
        frecuencyVector[j] = (samplingFrecuency*j)/(double)N;
    }

    for(k=0; k<N/2; k++) {

        magnitudeVector[k] = 2.0*(sqrt(pow(v[k].Re/(double)N, 2) + pow(v[k].Im/(double)N, 2)));

        if (  (frecuencyVector[k]>=(frecuenciesCoil[0]-deltaF) ) && (frecuencyVector[k]<=(frecuenciesCoil[0]+deltaF))  ){
            binsMatriz[0][m] = magnitudeVector[k];
        }
        if (  (frecuencyVector[k]>=(frecuenciesCoil[1]-deltaF) ) && (frecuencyVector[k]<=(frecuenciesCoil[1]+deltaF))  ){
            binsMatriz[1][m] = magnitudeVector[k];
        }
        if (  (frecuencyVector[k]>=(frecuenciesCoil[2]-deltaF) ) && (frecuencyVector[k]<=(frecuenciesCoil[2]+deltaF))  ){
            binsMatriz[2][m] = magnitudeVector[k];
        }

    }
}

void GibicTrack::print_vector(const char *title, complex *x, int n){
    int i;

    printf("%s (dim=%d):", title, n);

    for(i=0; i<n; i++ ) printf(" %5.2f,%5.2f ", x[i].Re,x[i].Im);

    putchar('\n');

    return;
}
