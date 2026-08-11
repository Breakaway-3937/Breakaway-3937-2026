package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.utility.QuestNavADBWatcher;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final QuestNavADBWatcher questnavADB = new QuestNavADBWatcher();
  private int pdhLoopCounter = 0;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    questnavADB.start();
    try {
      Logger.recordMetadata("ProjectName", "MyProject");
      if(isReal()) {
        Logger.addDataReceiver(new WPILOGWriter());
      }
      Logger.start();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    pdhLoopCounter++;


      robotContainer.logFullPDH();
    

    if (pdhLoopCounter % 50 == 0) {
      robotContainer.logPDHStickyFaults();
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}