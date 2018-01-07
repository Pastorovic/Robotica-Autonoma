#include <QCoreApplication>
#include <Aria.h>

#include <control.h>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    Aria::init();
    ArRobot robot;
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
    // and then loads parameter files for this robot.
    ArRobotConnector robotConnector(&parser, &robot);

    // Always try to connect to the first laser:
    // Comentar las dos siguientes líneas si no se va a tuilizar el laser
    ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
    parser.addDefaultArgument("-connectLaser");

    if(!robotConnector.connectRobot())
    {
      ArLog::log(ArLog::Terse, "Could not connect to the robot.");
      if(parser.checkHelpAndWarnUnparsed())
      {
          Aria::logOptions();
          Aria::exit(1);
          return 1;
      }
    }
    if (!Aria::parseArgs())
    {
      Aria::logOptions();
      Aria::exit(1);
      return 1;
    }

    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);

    puts("Press CTRL-C or Escape to exit.\n");

    ArLog::log(ArLog::Normal, "Connected.");

    ArSonarDevice sonar;
    robot.addRangeDevice(&sonar);

    // Start the robot processing cycle running in the background.
    // True parameter means that if the connection is lost, then the
    // run loop ends.
    robot.runAsync(true);

    // try to connect to laser. if fail, warn but continue, using sonar only
    if(!laserConnector.connectLasers())
    {
      ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only.");
    }

    Control control(&robot);

    control.execute();

    // exit
    robot.waitForRunExit();
    ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
    Aria::exit(0);
    
    return a.exec();
}
