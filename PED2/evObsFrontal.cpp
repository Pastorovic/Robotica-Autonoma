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

int main( int argc, char** argv ) {
    // Genera números aleatorios para elegir la dirección
    srand( time( NULL ) );
    int numero;           //Variable donde se guardará el número aleatorio
    // Distancia y ángulo medidos por el laser al detectar un obstáculo
    int distance;
    double angle;
    // Almacena los grados de rotación girados
    int rot = 0;
    ArTime start;         //Variable para contar tiempo en hacer una accion
    // Define un tipo que indica en que dirección ha ido el robot hasta el momento
    // empieza en dirección de frente
    enum directions { straight, left, right };
    enum directions goingTo;
    goingTo = straight;

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
        dist = ( *laser ).currentReadingPolar( -22.5, 22.5, &angle );

        if( distance < 1000 ){
            printf("Encontrado obstaculo a %2.3f metros en angulo %2.2f.\n", ( float )distance/1000, angle );
            robot.lock();
            robot.stop();
            robot.unlock();
        //Miramos cual ha sido la última acción
        if(ultima_accion==0){
        //Si venia de movimiento seleccionamos el próximo giro al azar
        numero = rand() % 10; //número aleatorio de 0 a 9 (10 números posibles)
        if(numero<5){
        puts("Vengo de moverme. Selecciono al azar izquierda...");
        rotacion = rotacion + 90;
        ultima_accion=1;
        }else{
        puts("Vengo de moverme. Selecciono al azar derecha...");
        rotacion = rotacion - 90;
        ultima_accion=2;
        }
        }else if(ultima_accion==1){
        puts("Vengo de girar a la izquierda, Giro otra a la izquierda.");
        rotacion=rotacion+90;
        }else{
        puts("Vengo girar a la derecha, giro otra vez a la derecha.");
        rotacion=rotacion-90;
        }
        robot.lock();
        robot.setHeading(rotacion);
        robot.unlock();
        start.setToNow();
        while (1){
        robot.lock();
        if (robot.isHeadingDone(5)){
        robot.unlock(); break;
        }
        if (start.mSecSince() > 5000){
        robot.unlock(); break;
        }
        robot.unlock();
        ArUtil::sleep(50);
        }
        ArUtil::sleep(100);
        }else{
        ultima_accion=0;    //Pongo el flag que me he movido
        robot.lock();
        robot.setVel(250);
        robot.unlock();
        }
        ArUtil::sleep(100);
    }
    return 0;
}
