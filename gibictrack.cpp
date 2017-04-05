#include "gibictrack.h"
#include <QtSerialPort/QSerialPort>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <QDebug>
#include "RaabAlgorithm.h"
#include <QTimer>

#define q    7        /* for 2^7 points --- Señal de 2^n datos */
#define N    (1<<q)        /* N-point FFT, iFFT */

#ifndef PI
#define PI    3.14159265358979323846264338327950288
#endif

bool GibicConectado = false;
int totalDatos = 0;
int total=0;
double arregloXYZ[80000][3];//Se inicializa en el constructor de la clase (mainwindow) al reservar espacio para el maximo de bytes esperados
double binsMatriz[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
double posXYZ [3][2];
double retorno[3][2] = {{0.0,0.0},{0.0,0.0},{0.0,0.0}};
const int samplingFrecuency = 80000;//80000;
const int deltaF=samplingFrecuency/(2*N);//El deltaF= fs/N pero para estrechar el intervalo en el que cae la frecuencia divido por 2, evitando coger 2 valores
//const int deltaF=50;
//Esto hay que organizarlo las frecuencias no estan en orden
const int frecuenciesCoil[3] = {5000,10000,15000};
uchar sendValues[7] = {1, 200, 300, 400, 5, 6, 7};
bool SlicerConectado=false;
bool SoketCreado=false;

//Variables IGTLink
igtl::Matrix4x4	matrix;
igtl::TransformMessage::Pointer trans_Msj;
igtl::PositionMessage::Pointer pos_Msj;
igtl::ClientSocket::Pointer mi_socket;

QByteArray vectorRX;

QTimer *timer;

// Definitions

#define sampleFreq	256.0f		// sample frequency in Hz
#define betaDef		1.0f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

GibicTrack::GibicTrack()
{
    vectorRX.reserve(150000);
    timer= new QTimer(this);
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
    //timer->start(500);
    return GibicConectado;
}

void GibicTrack::SolicitarDato(double retorno[3][2])
{
    qDebug() << "solicita dato !!!!";
    totalDatos = 0;
    serial->write("$");

    retorno[0][0] = posXYZ[0][0];
    retorno[1][0] = posXYZ[1][0];
    retorno[2][0] = posXYZ[2][0];

    sendValues[1] = posXYZ[0][0];
    sendValues[2] = posXYZ[1][0];
    sendValues[3] = posXYZ[2][0];

    sendValues[4] = q0;
    sendValues[5] = q1;
    sendValues[6] = q2;
    sendValues[7] = q3;
    qDebug("cuaternion");
    qDebug()<< q0 << q1 << q2 << q3;
    EmpaquetarDatos(sendValues);

}

void GibicTrack::SolicitarDatoIMU(){
    total = total + 1;
    qDebug()<< total;
    //serial->write("*");
  //MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    //MadgwickAHRSupdate(-0.9375, -1.25, 0.875, 0.019, -0.0522, 0.978, 0.21, 0.0313, -0.4487);
    //MadgwickAHRSupdate( 6.3750,  2.1875, 1.6875, 0.0034, -0.9868, -0.0474, 0.2178, 0.4292, 0.0703);
    MadgwickAHRSupdate(-6.5, 20, -4.25, 1.0586, 0.0098, 0.0176, -0.4307, -0.0181, -0.1904);
    //MadgwickAHRSupdate(2.5, 22.1875, 23.5, 0.019, -0.0093, 0.9819, 0.2051, -0.0405, -0.4463);
    //MadgwickAHRSupdate(-1.6250, -13.1250, 7.75, -0.0396, -0.0796, 0.9858, 0.2305, 0.00097656, -0.4414);

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
        PosOri_Raab (binsMatriz, posXYZ, radio);
        total = 0;
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

    //Conexion para la señal de finalizaci{on del conteo del timer
    connect(timer, SIGNAL(timeout()), this, SLOT(SolicitarDatoIMU()));
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

        //printf("\n");
        //print_vector("Orig", v, N);

        /* FFT of v[]: */

        fft( v, N, scratch );

        //printf("original\n");

        //qDebug()<< QString::number(i);


        //printf("calculo de FFT\n");

        //print_vector(" FFT", v, N);

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
            binsMatriz[m][0] = magnitudeVector[k];
        }
        if (  (frecuencyVector[k]>=(frecuenciesCoil[1]-deltaF) ) && (frecuencyVector[k]<=(frecuenciesCoil[1]+deltaF))  ){
            binsMatriz[m][1] = magnitudeVector[k];
        }
        if (  (frecuencyVector[k]>=(frecuenciesCoil[2]-deltaF) ) && (frecuencyVector[k]<=(frecuenciesCoil[2]+deltaF))  ){
            binsMatriz[m][2] = magnitudeVector[k];
        }
    }
    if(m==2){
        qDebug()<< binsMatriz[0][0];
        qDebug()<< binsMatriz[0][1];
        qDebug()<< binsMatriz[0][2];
        qDebug()<< binsMatriz[1][0];
        qDebug()<< binsMatriz[1][1];
        qDebug()<< binsMatriz[1][2];
        qDebug()<< binsMatriz[2][0];
        qDebug()<< binsMatriz[2][1];
        qDebug()<< binsMatriz[2][2];
    }
}

void GibicTrack::print_vector(const char *title, complex *x, int n){
    int i;

    printf("%s (dim=%d):", title, n);

    for(i=0; i<n; i++ ) printf(" %5.2f,%5.2f ", x[i].Re,x[i].Im);

    putchar('\n');

    return;
}


/*********************************************************
  FUNCION PARA ESTABLECER LA CONEXIÓN CON EL 3DSLICER
  ********************************************************/
void GibicTrack::Conectar3DSlicer(){
   if(!SlicerConectado)
        {
            qDebug()<< "3DSlicer: Conectando...\n";
//            /*Establecemos conexion con el servidor-> ip: 192.168.1.11, port number: 18944
//            que son los valores del "LocalHost" y el puerto por defecto de 3dSlicer
//            Usando las clase suministrada por la libreria OpenIGTLink -> igtl::ClientSocket*/
            if(!SoketCreado){
                mi_socket = igtl::ClientSocket::New();
                SoketCreado = true;
            }

            int result = mi_socket->ConnectToServer("localhost",18944);//192.168.1.11
            if (result != 0)//Chequeamos el resultado de la +QString::numberconexion
            {
               qDebug()<< "3DSlicer: Error\n";
            }else
            {
                SlicerConectado=true;
                qDebug()<< "3DSlicer: Conexion OK\n";
            }
//            /*Creamos la instancia de la clase de mensaje OpenIGTLink de acuerdo al
//            mensaje que vamos a enviar en este caso sera una transformación o una posición*/

                pos_Msj = igtl::PositionMessage::New();
//                /*|Indicamos que vamos a enviar tanto posicion como orientacion en el formato
//                de quaterniones y que vamos a enviar los 4 elementos*/
                pos_Msj->SetPackType(igtl::PositionMessage::ALL);
//                /*|Colocamos un nombre que identifique al dispositivo que envia la informacion via el protocolo
//                OpenIGTLink, esto es util si por ejemplo hay varios sensores enviando el mismo tipo de información*/
                pos_Msj->SetDeviceName("TrackerG1B1C");

        }else{
            SlicerConectado = false;
            mi_socket->CloseSocket();
            qDebug()<< "3DSlicer: Desconectado\n";
    }
}


/*****************************************************************************
  FUNCION PARA EMPAQUETAR LOS DATOS CON EL FORMATO ADECUADO PARA SU ENVIO
  ****************************************************************************/
void GibicTrack::EmpaquetarDatos(const uchar *datos)
{
    float* PosVec=new float[3];
    float* QtrnVec=new float[4];

    PosVec[0]=datos[1];
    PosVec[1]=datos[2];
    PosVec[2]=datos[3];
    //*******************************************************************
    //*******************************************************************
    //   O   J   O
    //*******************************************************************
    //*******************************************************************
    //Falta convertir los angulos de Euler en el quaternion respectivo
    qDebug()<< q0 << q1 << q2 << q3;
    QtrnVec[0]= q0;
    QtrnVec[1]= q1;
    QtrnVec[2]= q2;
    QtrnVec[3]= q3;
    qDebug()<< "datos vector quaternion";
    qDebug()<< QtrnVec[0] << QtrnVec[1] << QtrnVec[2] << QtrnVec[3];
    EnviarPosicion(PosVec,QtrnVec);
}

void GibicTrack::EnviarPosicion(float *PosVec, float *QtrnVec)
{
    Q_UNUSED (PosVec);
    Q_UNUSED (QtrnVec);
//    /*|"Empaquetamos" la información. Es decir utilizamos la funcion miembro "Pack" que
//    genera un flujo o estructura de bytes con el formato indicado por el protocolo OpenIGTLink*/
    pos_Msj->SetPosition(PosVec);
    pos_Msj->SetQuaternion(QtrnVec);
    pos_Msj->Pack();
//    /*|Enviamos la informacion a traves de la conexion por sockets TCP/IP utilizando la clase
//    igtl::ClientSocket la cual puede ser reemplazada por otra libreria que implemente sockets
//    como Csocket pero mejor esta que se construye directamente sobre win32socket*/
    mi_socket->Send(pos_Msj->GetPackPointer(), pos_Msj->GetPackSize());
}

//====================================================================================================
// Functions: Calgulo de orientacion a partir de componentes enviadas por IMU

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void GibicTrack::MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void GibicTrack::MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    qDebug("2");
    qDebug()<< q0 << q1 << q2 << q3;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float GibicTrack::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


