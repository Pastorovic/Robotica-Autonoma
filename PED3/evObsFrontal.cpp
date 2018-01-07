/***********************************************************
 *    Name:         evObsFrontal.cpp                       *
 *    Title:        Control simple del movimiento mediante *
 *                  percepción                             *
 *    Description:  El robot evita obstáculos girando      *
 *                  90 grados cuando encuentra uno         *
 *    Author:       David Pastor Sanz                      *
 ***********************************************************/

#include "Aria.h"

/********************************************************
 * Termina la ejecución devolviendo un mensaje de error *
 * si los valores de los configuración son erroneos     *
 * o el robot no se conecta correctamente               *
 ********************************************************/
void checkConnection( ArRobotConnector &connector ) {
  if( !connector.parseArgs() ) {
    ArLog::log( ArLog::Normal, "Valores de configuración erroneos" );
    Aria::shutdown();
    exit( 1 );
  }

  if( !connector.connectRobot() ) {
    ArLog::log( ArLog::Normal, "No ha sido posible realizar la conexión" );
    Aria::shutdown();
    exit( 1 );
  }

  return;
}

/***********************************************************
 * Devuelve la dirección a la que irá el robot, en caso de *
 * no haberse elegido dirección, la escoge a partir de un  *
 * número aleatorio mod 2, si el resultado es 0, gira a la *
 * izquierda, si es 1 a la derecha (par-izq, impar-der)    *
 ***********************************************************/
int choosePath( int &dir ) {
    if( dir == -1 ) {
        // Genera números aleatorios para elegir la dirección
        srand( time( NULL ) );
        dir = rand() % 2;
    }
    if( dir == 0 ) {
        ArLog::log( ArLog::Normal, "Girando 90 grados a izquierda..." );
        return -90;
    }
    else {
        ArLog::log( ArLog::Normal, "Girando 90 grados a derechas..." );
        return 90;
    }
}

/********************************************
 * Espera hasta que realiza un gir completo *
 * o hayan pasado 5 segundos                *
 ********************************************/
 void waitToTurn( ArRobot &robot ) {
     ArTime start;
     start.setToNow();
     while( 1 ){
         robot.lock();
         if( robot.isHeadingDone( 5 ) ) {
             robot.unlock();
             break;
         }
         if( start.mSecSince() > 5000 ) {
             robot.unlock();
             break;
         }
         robot.unlock();
         ArUtil::sleep( 50 );
     }
     ArUtil::sleep( 100 );
     return;
 }

int main( int argc, char** argv ) {
    // Genera números aleatorios para elegir la dirección
    srand( time( NULL ) );
    // Distancia y ángulo medidos por el laser al detectar un obstáculo
    int distance;
    double angle;
    // Almacena los grados de rotación girados
    int rot = 0;
    // Define la dirección a la que irá. -1 indica que empieza yendoi recto
    int dir = -1;
    // Lleva la cuenta del tiempo de una acción
    ArTime start;


    // inicializar la libreria de Aria
    Aria::init();

    // se instancian los argumentos del parser y a partir de este el conector
    ArArgumentParser parser( &argc, argv );
    // cargar los valores por defecto
    parser.loadDefaultArguments();
    // se crea un objeto robot para poder manejar en el simulador
    // únicamente se activan los actuadores del motor. Sin sensores conectados
    ArRobot robot;
    // se conectan el parser y el robot
    ArRobotConnector connector( &parser, &robot );
    // se conecta el laserAng
    ArLaserConnector lConnector( &parser, &robot, &connector);

    /* conexión del robot */

    // comprueba los argumentos y la conexión del robot
    checkConnection( connector );
    ArLog::log( ArLog::Normal, "Conexión establecida" );
    // activa el modo asíncrono
    robot.runAsync( true );

    // esperar hasta que se conecte el laser y que tome la primera medida
    ArLog::log( ArLog::Normal, "Encendiendo laser..." );
    while ( !lConnector.connectLasers() ) {
        ArUtil::sleep( 100 );
    }
    ArUtil::sleep( 2000 );
    // Selección del laser frontal
    ArLaser* laser = ( ArLaser* )( *robot.getRangeDeviceList() ).front();
    ArLog::log( ArLog::Normal, "Laser en funcionamiento" );

    ArLog::log( ArLog::Normal, "Encendiendo motores..." );
    // bloquea al robot durante la configuración
    robot.lock();
    // enciende los motores
    robot.enableMotors();
    // desbloquea el robot una vez está configurado
    robot.unlock();
    ArLog::log( ArLog::Normal, "Motores encendidos\n Comenzando la simulación..." );
    ArUtil::sleep( 500 );

    while( 1 ) {
        // Calcula la distancia más cercana en la que se encuentra un obstáculo
        // como solo se usa el laser frontal que abarca 45 grados se toma la medida
        // entre 0-(45/2) y 0+(45/2)
        // además almacena en angle el ángulo en el que se encuentra
        distance = ( *laser ).currentReadingPolar( -22.5, 22.5, &angle );

        // Si se encuentra un objeto a menos de 1 metro de distancia
        if( distance < 1000 ){
            ArLog::log( ArLog::Normal, "Obstáculo encontrado" );
            ArLog::log( ArLog::Normal, "  -Distancia: %.2f m", ( float )distance/1000 );
            ArLog::log( ArLog::Normal, "  -Angulo:    %.2f grados", angle );
            //Paramos el robot
            robot.lock();
            robot.stop();
            robot.unlock();
            // realizar la rotación
            rot += choosePath( dir );
            robot.lock();
            robot.setHeading( rot );
            robot.unlock();
            waitToTurn( robot );
        }
        // una vez no se tiene ningún obstáculo delante se sigue en línea recta
        else {
            // se vuelve a poner la dirección frontal
            dir = -1;
            robot.lock();
            robot.setVel( 300 );
            robot.unlock();
        }
        ArUtil::sleep(100);
    }
    return 0;
}
