#include "RaabAlgorithm.h"
#include <QDebug>

// Implementación del algoritmo de Raab para calcular la posición y la
// orientación a partir de una matriz con los voltajes de sensado
// Vsens     Matriz con los voltajes de sensado.
//               Fila->Bobina que sensa
//               Columna->Bobina que transmite
//               Ejem: Vsens(1,3) f=1,c=3 Voltaje en bobina X inducido por
//                       bobina transmisora Z

void PosOri_Raab (double binsMatriz[3][3], double posXYZ [3][2], double &radio){
    matriz3x3 Vsens;
    vector3x1 P;
    double r6;
    double r;
    vector3x1 CorN;
    vector3x1 xyz;
    double C=Cprom*2;

    //Copia local de la matriz para trabajarla como el tipo matriz3x3
    //Se traspone la matriz para que los valores almacenados correspondan a lo esperado en Vsens
    //Ver comentarios del algoritmo mas arriba "Vsens     Matriz con los voltajes de sensado."
    for (int fila = 0; fila < 3; ++fila)
        for (int colum = 0; colum < 3; ++colum)
            Vsens(fila,colum) = binsMatriz[colum][fila];



    for(int r=0;r<3;r++){
        P(r,0) = pow(Vsens(r,0),2)+pow(Vsens(r,1),2)+pow(Vsens(r,2),2);
    }

    //%11.4e Mostrar en notación cientifica con 11 caracteres y 4 digitos luego del punto
    printf("\n\nP =(%11.4e, %11.4e, %11.4e)\n", P(0,0),P(1,0),P(2,0));

    r6=(pow(C,2)/(2.0/3.0*(P(0,0)+P(1,0)+P(2,0))));
    r=pow(r6,(1.0/6.0));
    radio=r;
    printf("\n\nR =%20.4e\n", r);
    CorN=Np*Mp*P;

    //    Se calculan las coordenadas segun Raab:
    xyz=(pow(r,4)/C)*(raizElem(CorN));

    //printVector("XYZ Raab",xyz);
    posXYZ[0][0]=xyz(0,0);
    posXYZ[1][0]=xyz(1,0);
    posXYZ[2][0]=xyz(2,0);

    //    %%%%%%Calculo de angulos%%%%%%%%%
    vector3x1 alfa;
    vector3x1 beta;
    alfa(0,0)=atan2(xyz(1,0),xyz(0,0));
    alfa(1,0)=asin(xyz(1,0)/sqrt(pow(xyz(0,0),2)+pow(xyz(1,0),2)));
    alfa(2,0)=acos(xyz(0,0)/sqrt(pow(xyz(0,0),2)+pow(xyz(1,0),2)));

    beta(0,0)=atan2(xyz(2,0),sqrt(pow(xyz(0,0),2)+pow(xyz(1,0),2)));
    beta(1,0)=asin(xyz(2,0)/r);
    beta(2,0)=acos(sqrt(pow(xyz(0,0),2)+pow(xyz(1,0),2))/r);

    /*    %Angulos de posición (junto con r definen la posición), tome los que para
            %este caso dieron mejor, con el promedio no da tan bien*/
    double AlfaEst=alfa(2,0);
    double BetaEst=beta(0,0);

    printf("Alfa y Beta =(%11.4e, %11.4e)\n", AlfaEst*180/M_PI,BetaEst*180/M_PI);


    //Calculo alternativo de coordenadas al propuesto por Raab, al menos en la simulación da mas exacto
    double betaAlt=acos(sqrt(4.0/3.0*(1.0-r6/pow(C,2)*P(2,0))));
    double alfaAlt1=acos(sqrt(4.0/(3.0*pow(cos(betaAlt),2))*(r6/pow(C,2)*P(0,0)-1.0/4.0)));
    double alfaAlt2=asin(sqrt(4.0/(3.0*pow(cos(betaAlt),2))*(r6/pow(C,2)*P(1,0)-1.0/4.0)));

    //Identificar porque es necesario este if, este valor NUNCA deberia ser negativo, chequear teoria matematica del algoritmo
    double alfaAlt;
    if((r6/pow(C,2)*P(1,0)-1.0/4.0)>=0){
        alfaAlt=alfaAlt2;
    }else{
    alfaAlt=alfaAlt1;
    }

    double Alt_Z=r*sin(betaAlt);
    double Alt_Y=r*sin(alfaAlt)*cos(betaAlt);
    double Alt_X=r*cos(alfaAlt)*cos(betaAlt);

//    printf("\n\nXYZ Alternativos =(%11.4e, %11.4e, %11.4e)\n", Alt_X,Alt_Y,Alt_Z);
//    printf("Alfa y Beta =(%11.4e, %11.4e)\n", alfaAlt*180/M_PI,betaAlt*180/M_PI);
    posXYZ[0][1]=Alt_X;
    posXYZ[1][1]=Alt_Y;
    posXYZ[2][1]=Alt_Z;

    //Calculo de la matriz de rotación de la orientación (AER)
    matriz3x3 T_a=CalAzimuth_MatRot(AlfaEst);
    matriz3x3 T_b=CalElevation_MatRot(BetaEst);
    matriz3x3 T_ma=CalAzimuth_MatRot(-AlfaEst);
    matriz3x3 T_mb=CalElevation_MatRot(-BetaEst);

    /*Dado que las anteriores 4 matrices, excluyendo S_inv que se puede construir una sola vez en otra parte, se estaran creando
          cada vez que se requiera calcular la const posición, como minimo una vez por segundo, esta implementación puede traer muchos
          inconvemientes si se esta asignando en cada calculo una nueva posición de memoria sin liberar la anterior, aunque es posible
          que este no sea el caso si los metodos se llaman como parte de un objeto que se elimina para estar seguro cambiare esto por
          un par de metodos que se encargaran de llenar las matrices inicialmente creadas sin volverles a asignar un nuevo espacio.
          Tener en cuenta que si se garantiza la liberación de memoria quizas sea mejor usar esta creación insitu que la modificación
          de las matrices.*/

    matriz3x3 Q_inv=T_ma*T_mb*S_inv*T_b*T_a;

    matriz3x3 A=pow(r,3.0)/C*Vsens*Q_inv;

    //Convertir a la matriz de orientación a quaternion

/*    Usar o replicar el metodo:  "igtl::MatrixToQuaternion"
      Implementado en el archivo "igtlMath.h" junto con otros metodos utiles como impresión de matrices y vectores*/

}

vector3x1 raizElem(vector3x1 wVector){
    for(int i=0;i<3;i++){
        wVector(i,0)=sqrt(wVector(i,0));
    }
    return wVector;
}

/*Metodo para calcular la matriz de rotación ortogonal para una rotación alrededor del eje Z
  que torna (orienta) el eje X en el eje Y --> AZIMUTH*/
matriz3x3 CalAzimuth_MatRot(double angulo){
    matriz3x3 rotationMat;

    rotationMat(0,0)=cos(angulo);
    rotationMat(0,1)=sin(angulo);
    rotationMat(0,2)=0.0;

    rotationMat(1,0)=-sin(angulo);
    rotationMat(1,1)=cos(angulo);
    rotationMat(1,2)=0.0;

    rotationMat(2,0)=0.0;
    rotationMat(2,1)=0.0;
    rotationMat(2,2)=1.0;

    return rotationMat;
}

/*Metodo para calcular la matriz de rotación ortogonal para una rotación alrededor del eje Y
  que torna (orienta) el eje Z en el eje X --> AZIMUTH*/
matriz3x3 CalElevation_MatRot(double angulo){
    matriz3x3 rotationMat;

    rotationMat(0,0)=cos(angulo);
    rotationMat(0,1)=0.0;
    rotationMat(0,2)=-sin(angulo);

    rotationMat(1,0)=0.0;
    rotationMat(1,1)=1.0;
    rotationMat(1,2)=0.0;

    rotationMat(2,0)=sin(angulo);
    rotationMat(2,1)=0.0;
    rotationMat(2,2)=cos(angulo);

    return rotationMat;
}

void printMatriz(const char *texto, matriz3x3 impMatriz){
    printf("\n\n%s =\n   %11.4e, %11.4e, %11.4e\n   %11.4e, %11.4e, %11.4e\n   %11.4e, %11.4e, %11.4e\n", texto,
           impMatriz(0,0),impMatriz(0,1),impMatriz(0,2),
           impMatriz(1,0),impMatriz(1,1),impMatriz(1,2),
           impMatriz(2,0),impMatriz(2,1),impMatriz(2,2));
}

void printVector(const char *texto, vector3x1 impVector){
    printf("\n\n%s =(%11.4e, %11.4e, %11.4e)\n", texto,
           impVector(0,0),impVector(1,0),impVector(2,0));
}

