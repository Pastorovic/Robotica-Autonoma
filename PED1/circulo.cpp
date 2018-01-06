/**********************************************************
 *    Name:         circulo.cpp                           *
 *    Title:        Control de movimiento usand odometría *
 *    Description:  El robot se mueve describiendo un     *
 *                  círculo de dos metros de radio        *
 *    Author:       David Pastor Sanz                     *
 **********************************************************/

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

int main( int argc, char** argv ) {
  // variables necesarias
  // se realiza mediante el cálculo del perímetro de la circunferencia
  // y la velocidad a la que se desplaza el robot, así va girando en función
  // al tiempo que necesita
  const float PI = 3.1415;
  float radio = 2;
  float speed = 500;
  float perimeter = 2 * PI * radio;
  float time_needed = perimeter / ( speed / 1000 );
  float rotation = 360 / time_needed;

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

  /* realización del circulo */
  robot.lock();
  robot.setRotVel( rotation );
  robot.unlock();
  robot.lock();
  robot.setVel( speed );
  robot.unlock();
  ArUtil::sleep( time_needed * 1000 );
  robot.stop();

  ArLog::log( ArLog::Normal, "Simulación terminada" );

  // terminación de Aria
  Aria::exit( 0 );
  return 0;
}
