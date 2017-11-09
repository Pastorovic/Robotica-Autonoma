/**********************************************************
 *    Name:         cuadrado.cpp                          *
 *    Title:        Control de movimiento usand odometría *
 *    Description:  El robot se mueve describiendo un     *
 *                  cuadrado de 4 metros de lado          *
 *    Author:       David Pastor Sanz                     *
 **********************************************************/

#include "/usr/local/Aria/include/Aria.h"
#include <stdio.h>

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

int main( int argc, char** argv ) {
  // grados de giro y cantidad de traslación
  int rot = 90;
  int tra = 4000;

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

  /* conexión del robot */

  // comprueba los argumentos y la conexión del robot
  checkConnection( connector );
  ArLog::log( ArLog::Normal, "Conexión establecida" );
  // activa el modo asíncrono
  robot.runAsync( true );

  ArLog::log( ArLog::Normal, "Encendiendo motores..." );
  // bloquea al robot durante la configuración
  robot.lock();
  // enciende los motores
  robot.enableMotors();
  // desbloquea el robot una vez está configurado
  robot.unlock();
  ArLog::log( ArLog::Normal, "Motores encendidos\n Comenzando la simulación..." );
  ArUtil::sleep( 500 );

  /* realización del cuadrado */
  for( int i = 0; i < 4; i++ ) {
    // movimiento de traslación
    ArLog::log( ArLog::Normal, "Moviéndose 4 metros..." );
    robot.lock();
    robot.move( tra );
    robot.unlock();
    // no continúa hasta que se haya realizado completamente el movimiento
    while( 1 ) {
      robot.lock();
      if( robot.isMoveDone() ) {
        robot.unlock();
        break;
      }
      robot.unlock();
      ArUtil::sleep( 150 );
    }
    ArUtil::sleep( 2000 );

    // movimiento de rotación
    ArLog::log( ArLog::Normal, "Girando 90 grados..." );
    robot.lock();
    robot.setHeading( rot );
    robot.unlock();
    // no continúa hasta que se haya realizado completamente el giro
    while( 1 ) {
      robot.lock();
      if( robot.isHeadingDone() ) {
        robot.unlock();
        break;
      }
      robot.unlock();
      ArUtil::sleep( 150 );
    }
    ArUtil::sleep( 2000 );
    rot += 90;
  }
  ArLog::log( ArLog::Normal, "Simulación terminada" );

  // terminación de Aria
  Aria::exit( 0 );
  return 0;
}
