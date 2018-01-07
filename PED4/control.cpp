#include "control.h"

// Definición de variables y constantes
// Valor máximo que puede medir el laser
const double MAX = 32000;
// Entradas de los sensores. Cada uno cubrirá 2 grados.
// Se han ido probando distintos valores para esto hasta encontrar el que mejor se ajusta, a mayor rango más se mezclaban
// cada uno de las entradas de los sensores y el resultado no era el deseado, realizando movimientos bruscos y sin sentido
double sensorReadings[5];
// Lista de umbrales
double thresholdList[5];
// Matriz de pesos. La primera lista representa los sensores, la segunda las ruedas (posición 0, derecha; 1, izquierda).
double weight[5][2];
// Lista de salidas para los motores
double motorOutput[2];
// Velocidad máxima
double maxSpeed;

Control::Control(ArRobot *robot) : robot(robot)
{
    init();
}

void Control::init()
{
    robot->lock();
    // Activar motores
    robot->enableMotors();

    // Establecer laser y sonar,
    // si no está alguno será = 0
    laser = robot->findLaser(1);
    sonar = (ArSonarDevice*)robot->findRangeDevice("sonar");

    // MobileSim no devuelve correctamente este valor si no se establece antes
    robot->setTransVelMax( 600 );

    robot->unlock();

    // Lista de umbrales en milímetros
    thresholdList[0] = 600;     // más a la izquierda
    thresholdList[1] = 800;
    thresholdList[2] = 1000;    // En frente
    thresholdList[3] = 800;
    thresholdList[4] = 600;     // más a la derecha

    // Lista de pesos
    // en el caso de encontrar obstáculo delante la rueda delantera derecha girara en sentido contrario a la izquierda para poder realizar un giro
    // se han ido jugando con los pesos hasta encontrar esta configuración que se ha creido la más adecuada
    weight[0][0] =  1;
    weight[0][1] =  5;
    weight[1][0] =  -1;
    weight[1][1] =  6;
    weight[2][0] =  5;
    weight[2][1] =-10;
    weight[3][0] =  6;
    weight[3][1] =  -1;
    weight[4][0] =  5;
    weight[4][1] =  1;
}


void Control::execute()
{
    // Bucle de control
    while(true) {
        input();
        proccess();
        output();
        ArUtil::sleep(100);
    }
}

void Control::input()
{
    if(laser) {
        laser->lockDevice();
        sensorReadings[0] = laser->currentReadingPolar( -90, -88, NULL );
        sensorReadings[1] = laser->currentReadingPolar( -46, -44, NULL );
        sensorReadings[2] = laser->currentReadingPolar( -1, 1, NULL );
        sensorReadings[3] = laser->currentReadingPolar( 44, 46, NULL );
        sensorReadings[4] = laser->currentReadingPolar( 88, 90, NULL );
        laser->unlockDevice();
    }
    if(sonar) {
        // TODO Lectura del sonar, no hay lock
        sonar->getCurrentBufferAsVector();
    }

    // Normalizar entradas de 0 a 1
    for( int i = 0; i <= 4; i++ ) {
        if( sensorReadings[i] > thresholdList[i] ) {
            sensorReadings[i] = 0;
        }
        else {
            sensorReadings[i] = 1 - ( sensorReadings[1] / MAX );
        }
    }
}

void Control::proccess()
{
    // Inicio de ruedas paradas
    motorOutput[0] = 0;
    motorOutput[1] = 0;

    // Calcular salidas a partir de entradas y otros posibles parámetros (pesos, reglas, etc)
    // rueda izquierda
    for( int i = 0; i <= 4; i++ ) {
        motorOutput[0] += ( sensorReadings[i] * weight[i][0] );
    }
    // rueda derecha
    for( int i = 0; i <= 4; i++ ) {
        motorOutput[1] += ( sensorReadings[i] * weight[i][1] );
    }

    // Normalizar salidas entre -1 y 1, el máximo peso es 5
    motorOutput[0] /= 5;
    motorOutput[1] /= 5;

    // Se suma la constante (se ha decidido por 0.5) si no existe ninguna lectura por debajo del umbral
    if( sensorReadings[0] + sensorReadings[1] + sensorReadings[2] + sensorReadings[3] + sensorReadings[4] == 0 ) {
        motorOutput[0] += 0.5;
        motorOutput[1] += 0.5;
    }
}

void Control::output()
{
    // Pasar salidas a velocidades de ruedas o velocidades de traslación y rotación
    maxSpeed = robot->getTransVelMax();
    robot->lock();
    robot->setVel2( motorOutput[0] * maxSpeed, motorOutput[1] * maxSpeed );
    robot->unlock();
}


