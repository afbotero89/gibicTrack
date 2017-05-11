#include "gibictrack.h"
#include <QtSerialPort/QSerialPort>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <QDebug>
#include "RaabAlgorithm.h"
#include <QTimer>
#include <QStringList>
#include <QDateTime>

#define q    9        /* for 2^7 points --- Señal de 2^n datos */
#define N    (1<<q)        /* N-point FFT, iFFT */

#ifndef PI
# define PI    3.14159265358979323846264338327950288
#endif

bool GibicConectado = false;
bool SlicerConectado = false;
bool SoketCreado = false;

int totalDatos = 0;
int total=0;
double arregloXYZ[80000][3];//Se inicializa en el constructor de la clase (mainwindow) al reservar espacio para el maximo de bytes esperados
double binsMatriz[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
double posXYZ [3][2];
double retorno[3][2] = {{0.0,0.0},{0.0,0.0},{0.0,0.0}};
const int samplingFrecuency = 25000;//80000;
const int deltaF=samplingFrecuency/(2*N);//El deltaF= fs/N pero para estrechar el intervalo en el que cae la frecuencia divido por 2, evitando coger 2 valores
//Esto hay que organizarlo las frecuencias no estan en orden
const int frecuenciesCoil[3] = {5000,10000,15000};
uchar sendValues[7]={1,200,100,15,6,7,2};
double signalRx[128];
double magVec[N/2];
double frecVec[N/2];

//Variables IGTLink
igtl::Matrix4x4	matrix;
igtl::TransformMessage::Pointer trans_Msj;
igtl::PositionMessage::Pointer pos_Msj;
igtl::ClientSocket::Pointer mi_socket;

QByteArray vectorRX;
QTimer *timer;
QDateTime *currentTime;
QString vectorDatos;

// Variables madgwick (algoritmo para calculo de orientacion a partir de la IMU, algoritmo de fusion)
#define sampleFreq 100.0f                   // sample frequency in Hz
#define betaDef     0.38f                  // 2*proportional gain


volatile float beta = betaDef;								// 2 * proportional gain (Kp)


// q0: Escalar

// q1: Quaternion en X

// q2: Quaternion en Y

// q3: Quaternion en Z

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame


float vectorString_IMU[] = {0.0,0.0,0.0,0.0,0.0,0.0};

float ax = 0.0;
float ay = 0.0;
float az = 0.0;


float gx = 0.0;
float gy = 0.0;
float gz = 0.0;


float mx = 0.0;
float my = 0.0;
float mz = 0.0;

int filtroN = 0;

float contador = 0.1;

#define twoKpDef	(2.0f * 0.05f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

int cantidadDatosIMU=36;

GibicTrack::GibicTrack()
{
    vectorRX.reserve(150000);
    timer = new QTimer(this);
    currentTime = new QDateTime();
    qDebug()<< currentTime->currentDateTime();
}

void GibicTrack::SolicitarDatoIMUTimer(){
    serial->write("*");
    timer->start(10);
    //solicitaDatoIMU();
}

void GibicTrack::solicitaDatoIMU(){
    serial->write("*");
}

void GibicTrack::solicitaSenalesTimer(double magVec_graph[128/2], double frecVec_graph[128/2]){
    solicitaSenales();
    magVec_graph = magVec;
    frecVec_graph = frecVec;
}

void GibicTrack::solicitaSenales(){
    serial->write("$");
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

    //connect(serial, SIGNAL(readyRead()), this, SLOT(procesarDatos()));
    serial = new QSerialPort(this);
    serial->setPortName("/dev/ttyUSB0");//Se requiere saber con anterioridad el nombre asignado al puerto
    serial->setBaudRate(12000000);
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

void GibicTrack::SolicitarDato(double retorno[3][2])
{
    qDebug() << "pide dato";
    totalDatos = 0;
    //serial->write("*");
    retorno[0][0] = posXYZ[0][0];
    retorno[1][0] = posXYZ[1][0];
    retorno[2][0] = posXYZ[2][0];
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

void GibicTrack::readData()
{
    int cantidad=serial->bytesAvailable();
    QByteArray Qdata;
    if(cantidad==cantidadDatosIMU){
        Qdata = serial->readAll();
        //readDataIMU(total,Qdata);
        readQuaternions(Qdata);
        total = 0;
    }else{
        qDebug()<< "cantidad"<<cantidad;
        serial->flush();
    }



    /*Son 3840 para mandar 2 periodos de la señal fundamental cufa frecuencia es 250Hz, muestreando a 80kHz un periodo serian 320 datos,
     * dos periodos 640, por tres canales serian 1920 datos, y cada dato de 2 bytes da un total de 3840 bytes
     */

    /*
    if(total>=3840){//if(total>=N*3*2){
        //Se requiere de este casting para tomar los bytes del QByteArray como char sin signo
        const uchar *datosRX= reinterpret_cast<const uchar*>(vectorRX.constData());
        OrganizarDatos(datosRX);
        RealizarFFTs();
        //MostrarMatrizMag();
        PosOri_Raab (binsMatriz, posXYZ, radio);

        //MostrarXYZ(posXYZ);
        qDebug()<< posXYZ[0][0] << posXYZ[0][1] << posXYZ[0][2];
        sendValues[1] = posXYZ[0][0];
        sendValues[2] = posXYZ[0][1];
        sendValues[3] = posXYZ[0][2];
        EmpaquetarDatos(sendValues);

        total = 0;
//        console->putData(txtCant);

    }*/


    //    QString txtCant="No: "+QString::number(cantidad)+"->"+QString::number(total)+":  ";
    //    console->putData(txtCant);

    //TODO
    //Toda esta parte movida, para que se ejecute solo cuando este lleno el vector de recepción
    //DE AQUI
    //    const uchar *data= reinterpret_cast<const uchar*>(Qdata.constData());
    //    /*Para mostrar el dato suponiendo que el arreglo de bytes tiene la siguiente estructura:
    //      [0] -> Numero de la muestra
    //      [1] -> Posicion en X
    //      [2] -> Posicion en Y
    //      [3] -> Posicion en Z
    //      [4] -> Angulo de Azimut
    //      [5] -> Angulo de Elevación
    //      [6] -> Angulo de Roll
    //      */
    //    /*De esta forma esta suponiendo que siempre recibe 7 bytes si recibe un numero diferente se
    //      mostraran resultados incoherentes*/
    ////    MostrarDatos(data);
    ////    console->putData(data);

    //    OrganizarDatos(data);
    //    if(SlicerConectado)
    //        EmpaquetarDatos(data);

    //A AQUI
}

void GibicTrack::readDataIMU(int total, QByteArray Qdata){

//qDebug()<< vectorString_IMU[0]<< vectorString_IMU[1]<< vectorString_IMU[2]<< vectorString_IMU[3]<< vectorString_IMU[4]<< vectorString_IMU[5]<< vectorString_IMU[6];

        float floatValue[9];
        binaryToFloat(&floatValue[0], Qdata);

        ax = floatValue[0];
        ay = floatValue[1];
        az = floatValue[2];

        gx = floatValue[3];
        gy = floatValue[4];
        gz = floatValue[5];

        mx = floatValue[6];
        my = floatValue[7];
        mz = floatValue[8];

        for (int i =0;i<9;i++){
          qDebug()<<"float"<< floatValue[i];
        }
        gx *= 0.0174533f;
        gy *= 0.0174533f;
        gz *= 0.0174533f;
        if(gx==0.0f || gy == 0.0f || gz==0.0f || ax == 0.0f || ay == 0.0f || az==0.0f || mx == 0.0f || my == 0.0f || mz==0.0f){
            return;
        }else{

            MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, my, mx, -mz);
            EmpaquetarDatos(sendValues);
        }
        //MadgwickAHRSupdate(gx, -gy, -gz, -ax, ay, az, my, -mx, mz);
        //MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, my, mx, mz);


        ax = 0.0;
        ay = 0.0;
        az = 0.0;

        gx = 0.0;
        gy = 0.0;
        gz = 0.0;

        mx = 0.0;
        my = 0.0;
        mz = 0.0;

/*
        std::string datos = Qdata.constData();
        QString datos1 = QString::fromUtf8(datos.c_str());
        vectorDatos.append(datos1);

        int qStringCount = vectorDatos.split(",").count();
        if (qStringCount == 7){

            if(vectorDatos.split(",")[6]=="\r\n"){

                QStringList array = vectorDatos.split(",");

                ax = ax + array[0].toFloat();
                ay = ay + array[1].toFloat();
                az = az + array[2].toFloat();

                gx = gx + array[3].toFloat();
                gy = gy + array[4].toFloat();
                gz = gz + array[5].toFloat();
                //MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)

                filtroN = filtroN + 1;

                float filtro = 1.0;

                gx *= 0.0174533f;
                gy *= 0.0174533f;
                gz *= 0.0174533f;

                //MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, 0.0, 0.0, 0.0);
                //MadgwickAHRSupdate(1.566, 0.214, 0.279, 0.013, -0.017, -0.985, 0.0, 0.0, 0.0);
                //MadgwickAHRSupdate(-1.566, -0.214, -0.279, -0.013, 0.017, 0.985, 0.0, 0.0, 0.0);
                MahonyAHRSupdate(gx, gy, gz, ax, ay, az, 0.0, 0.0, 0.0);
                //qDebug()<< ax/filtro << ay/filtro << az/filtro << gx/filtro << gy/filtro << gz/filtro;
                EmpaquetarDatos(sendValues);
                filtroN = 0;

                ax = 0.0;
                ay = 0.0;
                az = 0.0;

                gx = 0.0;
                gy = 0.0;
                gz = 0.0;

                total= 0;

                vectorDatos.clear();
            }
        }*/
}

void GibicTrack::readQuaternions(QByteArray Qdata){
    float floatValue[9];
    binaryToFloat(&floatValue[0], Qdata);
    q0 = floatValue[0];
    q1 = floatValue[1];
    q2 = floatValue[2];
    q3 = floatValue[3];
    EmpaquetarDatos(sendValues);
}

void GibicTrack::binaryToFloat(float *array, QByteArray Qdata){
    uchar *p8 = (uchar *)array;
    for(int i = 0; i<cantidadDatosIMU; i++){

        *(p8 + i) = Qdata[i];
    }
}

void GibicTrack::closeSerialPort()
{
    if (serial->isOpen())
        serial->close();
}

void GibicTrack::initActionsConnections()
{
    //Conexiones para las señales del puerto serial
    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(handleError(QSerialPort::SerialPortError)));
    connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
    connect(timer,SIGNAL(timeout()),this,SLOT(solicitaDatoIMU()));
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

        //print_vector("Orig", v, N);

        /* FFT of v[]: */

        fft( v, N, scratch );

        //print_vector(" FFT", v, N);

        /* Get magnitude vector */

        getMagnitudeVector(v, binsMatriz, magVec, frecVec,i);

        qDebug()<< "magnigtud:"<< magVec;
        qDebug()<< "frecuecia:"<< frecVec;
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
        if (cntPos == 640)
        {
            cntVec++;
            cntPos=0;
        }
    }
}

// Funciones para calculo de FFT
void GibicTrack::RealizarFFT(double *signalRx){

    //console->clear();

    complex v[N], scratch[N];

    int k;

    /* Fill v[] with a function of known FFT: */

    for(int i=0;i<3;i++){
        for(k=0; k<N; k++) {
            signalRx[k]=arregloXYZ[k][i];//sig_10_10_05[k];
            v[k].Re = arregloXYZ[k][i];
            v[k].Im = 0;

        }

        //print_vector("Orig", v, N);

        /* FFT of v[]: */

        fft( v, N, scratch );

        //print_vector(" FFT", v, N);

        /* Get magnitude vector */

        double magVec[N/2], frecVec[N/2];
        getMagnitudeVector(v, binsMatriz, magVec, frecVec,i);

        //Para no graficar nivel DC que al ser ondas sobre 0V siempre tienen un nivel DC alto
        magVec[0]=0;

        //GraficarSenal(signalRx, GrapSenal[i]);
        //GraficarFFT(magVec, frecVec, GrapFFT[i]);

    }
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

void GibicTrack::Conectar3DSlicer(){
    if(!SlicerConectado)
        {
//            console->putData("3DSlicer: Conectando...\n");
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
                qDebug()<<"3DSlicer: Error\n";
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
           qDebug()<<"3DSlicer: Desconectado\n";
    }
}

/*****************************************************************************

  FUNCION PARA EMPAQUETAR LOS DATOS CON EL FORMATO ADECUADO PARA SU ENVIO

  ****************************************************************************/

void GibicTrack::EmpaquetarDatos(const uchar *datos)
{
    contador = contador + 0.1;
    float* PosVec=new float[3];
    float* QtrnVec=new float[4];
    float x = datos[1];
    float y = datos[2];
    float z = datos[3];

    PosVec[0]=0.0;
    PosVec[1]=0.0;
    PosVec[2]=0.0;

    //*******************************************************************

    //*******************************************************************

    //   O   J   O

    //*******************************************************************

    //*******************************************************************

    //Falta convertir los angulos de Euler en el quaternion respectivo

    qDebug()<<"x:" << q1 << "y:" << q2 << "z:" << q3 << "escalar:" << q0;

    QtrnVec[0]= q1;
    QtrnVec[1]= q2;
    QtrnVec[2]= q3;
    QtrnVec[3]= q0;

    EnviarPosicion(PosVec,QtrnVec);

}


void GibicTrack::EnviarPosicion(float *PosVec, float *QtrnVec)

{

    pos_Msj = igtl::PositionMessage::New();

//                /*|Indicamos que vamos a enviar tanto posicion como orientacion en el formato

//                de quaterniones y que vamos a enviar los 4 elementos*/

    pos_Msj->SetPackType(igtl::PositionMessage::ALL);

//                /*|Colocamos un nombre que identifique al dispositivo que envia la informacion via el protocolo

//                OpenIGTLink, esto es util si por ejemplo hay varios sensores enviando el mismo tipo de información*/

    pos_Msj->SetDeviceName("TrackerG1B1C");


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
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _8bx, _8bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

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
        recipNorm = 1.0f/sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = 1.0f/sqrt(mx * mx + my * my + mz * mz); //invSqrt(mx * mx + my * my + mz * mz);
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
        _8bx = 2.0f * _4bx;
        _8bz = 2.0f * _4bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        recipNorm = 1.0f/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
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
    recipNorm = 1.0f/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); //invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
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

    // Convert gyroscope degrees/sec to radians/sec

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * ((-q1 * gx) - (q2 * gy) - (q3 * gz));
    qDot2 = 0.5f * ((q0 * gx) + (q2 * gz) - (q3 * gy));
    qDot3 = 0.5f * ((q0 * gy) - (q1 * gz) + (q3 * gx));
    qDot4 = 0.5f * ((q0 * gz) + (q1 * gy) - (q2 * gx));

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = 1.0f/sqrt((ax * ax) + (ay * ay) + (az * az));
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
        s0 = (_4q0 * q2q2) + (_2q2 * ax) + (_4q0 * q1q1) - (_2q1 * ay);
        s1 = (_4q1 * q3q3) - (_2q3 * ax) + (4.0f * q0q0 * q1) - (_2q0 * ay) - _4q1 + (_8q1 * q1q1) + (_8q1 * q2q2) + (_4q1 * az);
        s2 = (4.0f * q0q0 * q2) + (_2q0 * ax) + (_4q2 * q3q3) - (_2q3 * ay) - _4q2 + (_8q2 * q1q1) + (_8q2 * q2q2) + (_4q2 * az);
        s3 = (4.0f * q1q1 * q3) - (_2q1 * ax) + (4.0f * q2q2 * q3) - (_2q2 * ay);
        recipNorm = 1.0f/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude

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
    recipNorm = 1.0f/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    //anglesComputed = 0;

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

// Mahony algorithm

void GibicTrack::MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

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
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void GibicTrack::MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
