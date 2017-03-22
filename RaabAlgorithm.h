#ifndef RAABALGORITHM_H
#define RAABALGORITHM_H

#include <QtGui/qgenericmatrix.h>
#include <qmath.h>



typedef QGenericMatrix<3,3,double> matriz3x3;
typedef QGenericMatrix<1,3,double> vector3x1;//1 columna, 3 filas

// Cs         Constates que agrupan parametros como la corriente y el numero de
//            vueltas de la bobina transmisora, y ganancia del amplificador de
//            sensado, se supone igual para una fuente con cada bobina receptora
const double Cprom=7903467.079375;//Promedio de Cx, Cy y Cz
//const double Cx=;
//const double Cy=;
//const double Cz=;


const double a1[9]= {1.0,    0.0,    0.0,
                     0.0,   -0.5,    0.0,
                     0.0,    0.0,   -0.5};

//private:

//las siguientes inicializaciones de matrices pueden poner problema dependiendo de la implementación de C++
    const matriz3x3 S(a1);

    const matriz3x3 Mp(new double[9]{5,    -1,    -1,
                                    -1,     5,    -1,
                                    -1,    -1,     5});

    const double Np=2.0/9.0;

    const matriz3x3 S_inv(new double[9]{1.0,    0.0,    0.0,
                                        0.0,   -2.0,    0.0,
                                        0.0,    0.0,   -2.0});


    vector3x1 raizElem(vector3x1 wVector);
    matriz3x3 CalAzimuth_MatRot(double angulo);
    matriz3x3 CalElevation_MatRot(double angulo);
    void printMatriz(const char *texto, matriz3x3 impMatriz);
    void printVector(const char *texto, vector3x1 impVector);

//public:
    void PosOri_Raab (double binsMatriz[3][3], double posXYZ[3][2], double &radio);


#endif // RAABALGORITHM_H
