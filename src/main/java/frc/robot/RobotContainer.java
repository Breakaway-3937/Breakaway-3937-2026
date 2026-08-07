package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;



import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SuperSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.utility.Constants;

public class RobotContainer {

  // Driver Controllers
  private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
  private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());
  private final Joystick buttons = new Joystick(Constants.Controllers.BUTTONS.getPort());
  private final JoystickButton topButton = new JoystickButton(buttons, Constants.Controllers.TOP_BUTTON);
  private final JoystickButton leftButton = new JoystickButton(buttons, 5);
  private final JoystickButton rightButton = new JoystickButton(buttons, 8);
  private final CommandXboxController xboxController = new CommandXboxController(
      Constants.Controllers.XBOX_CONTROLLER.getPort());

  // Subsystems
  private final PowerDistribution pdh = new PowerDistribution(27, ModuleType.kRev);
  
  private final Swerve s_Swerve = createSwerve();
  private final Shooter s_Shooter = new Shooter(pdh);
  private final Vision s_Vision = new Vision(s_Swerve, s_Shooter);
  private final Indexer s_Indexer = new Indexer(pdh);
  private final Intake s_Intake = new Intake(pdh);

  private final SuperSubsystem s_SuperSubsystem = new SuperSubsystem(s_Shooter, s_Indexer, s_Intake, s_Vision);

  // Misc
  private final SendableChooser<Command> autoChooser;
  private static double translationMultiplier = 1.0;
  private double rotationMultiplier = 1.0;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.MAX_SPEED * Constants.Controllers.STICK_DEADBAND)
      .withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * Constants.Controllers.STICK_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public RobotContainer() {

    NamedCommands.registerCommand("Shoot", s_SuperSubsystem.auto_Fire());
    NamedCommands.registerCommand("Combo", s_SuperSubsystem.auto_Combo());
    NamedCommands.registerCommand("Idle", s_SuperSubsystem.idle());
    NamedCommands.registerCommand("Intake", s_SuperSubsystem.auto_Intake());
    // NamedCommands.registerCommand("Climb", s_SuperSubsystem.climbRungOne());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.close();
    autoChooser.setDefaultOption("Papa Smurf Jeffords", Commands.none());
    autoChooser.addOption("Just Depot", new PathPlannerAuto("Just Depot").withName("Just Depot"));
    autoChooser.addOption("Just Human Player", new PathPlannerAuto("Just Human Player").withName("Just Human Player"));
    autoChooser.addOption("Depot + Human Player",
        new PathPlannerAuto("Depot + Human Player").withName("Depot + Human Player"));
    autoChooser.addOption("Arch P Left", new PathPlannerAuto("P Arch Long", false).withName("P Arch Long"));
    autoChooser.addOption("Arch P Right", new PathPlannerAuto("P Arch Long", true).withName("P Arch Long"));
    autoChooser.addOption("Arch Human Player",
        new PathPlannerAuto("Arch Human Player", false).withName("Arch Human Player"));
    // autoChooser.addOption("Double Short Sweep Left", new PathPlannerAuto("Short
    // Double Sweep", false).withName("Short Double Sweep"));
    // autoChooser.addOption("Double Short Sweep Right", new PathPlannerAuto("Arch
    // Depot", true).withName("Arch Depot"));
    autoChooser.addOption("Arch Depot Short", new PathPlannerAuto("Arch Depot", false).withName("Arch Depot"));
    autoChooser.addOption("Arch Depot Long", new PathPlannerAuto("Arch Depot Long", false).withName("Arch Depot Long"));
    // autoChooser.addOption("Anti Scream P Left", new PathPlannerAuto("Anti Scream
    // P", false).withName("Anti Scream P"));
    // autoChooser.addOption("Anti Scream P Right", new PathPlannerAuto("Anti Scream
    // P", true).withName("Anti Scream P"));
    autoChooser.addOption("P Arch Bump Cross Left",
        new PathPlannerAuto("P Arch Bump Cross", false).withName("P Arch Bump Cross"));
    autoChooser.addOption("P Arch Bump Cross Right",
        new PathPlannerAuto("P Arch Bump Cross", true).withName("P Arch Bump Cross"));
    autoChooser.addOption("Little P Left", new PathPlannerAuto("Little P Arch", false).withName("Little P Arch"));
    autoChooser.addOption("Little P Right", new PathPlannerAuto("Little P Arch", true).withName("Little P Arch"));
    // autoChooser.addOption("Test Combo", new PathPlannerAuto("Test
    // Combo").withName("Test Combo"));
    autoChooser.addOption("Delayed P Left", new PathPlannerAuto("Delayed P", false).withName("Delayed P"));
    autoChooser.addOption("Delayed P Right", new PathPlannerAuto("Delayed P", true).withName("Delayed P"));
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    s_Swerve.setDefaultCommand(
        s_Swerve.applyRequest(() -> drive
            .withVelocityX(translationController.getY() * translationMultiplier * Constants.Swerve.MAX_SPEED)
            .withVelocityY(translationController.getX() * translationMultiplier * Constants.Swerve.MAX_SPEED)
            .withRotationalRate(
                rotationController.getX() * rotationMultiplier * Constants.Swerve.MAX_ANGULAR_RATE)));

    xboxController.a().onTrue(s_SuperSubsystem.autoTrack(true));
    xboxController.y().onTrue(s_SuperSubsystem.autoTrack(false));
    xboxController.leftBumper().onTrue(s_SuperSubsystem.unclog()).onFalse(s_SuperSubsystem.idleWithIntakeDown());
    xboxController.leftBumper().whileTrue(s_SuperSubsystem.unclog());
    xboxController.leftTrigger(0.3).and(xboxController.rightTrigger(0.3).negate()).whileTrue(s_SuperSubsystem.intake());
    xboxController.rightTrigger(0.3).and(xboxController.leftTrigger(0.3).negate()).whileTrue(s_SuperSubsystem.fire());
    xboxController.leftTrigger(0.3).and(xboxController.rightTrigger(0.3)).whileTrue(s_SuperSubsystem.combo());
    xboxController.rightBumper().whileTrue(s_SuperSubsystem.shunClog());
    xboxController.rightStick().onTrue(s_SuperSubsystem.protectIntake());
    topButton.onTrue(s_Vision.setInitPose().ignoringDisable(true));
    leftButton.onTrue(s_Vision.setSpecialInitPose(true).ignoringDisable(true));
    rightButton.onTrue(s_Vision.setSpecialInitPose(false).ignoringDisable(true));
    xboxController.rightBumper().onTrue(s_SuperSubsystem.shunClog().repeatedly()).onFalse(s_SuperSubsystem.idleWithIntakeDown());
    // xboxController.povUp().onTrue(s_SuperSubsystem.raiseClimber());
    // xboxController.povLeft().onTrue(s_SuperSubsystem.stowClimber());
    // xboxController.povDown().onTrue(s_SuperSubsystem.pullClimber());

  }

  public static void applyMultipliers(double newMultiplier) {
    translationMultiplier = newMultiplier;
  }

  public Command getAutonomousCommand() {
    SmartDashboard.putString("Current Auto", autoChooser.getSelected().getName());
    return autoChooser.getSelected();
  }

  public Swerve getSwerve() {
    return s_Swerve;
  }

  public Shooter getShooter() {
    return s_Shooter;
  }

  public Indexer getIndexer() {
    return s_Indexer;
  }

  public Intake getIntake() {
    return s_Intake;
  }

  public Vision getVision() {
    return s_Vision;
  }

  /*
   * public Climber getClimber() {
   * return s_Climber;
   * }
   */
public void logFullPDH() {
SmartDashboard.putNumber("Robot Total Current Amps", pdh.getTotalCurrent());
SmartDashboard.putNumber("Robot Total Power Watts", pdh.getTotalPower());
SmartDashboard.putNumber("Robot Total Energy Joules", pdh.getTotalEnergy());
SmartDashboard.putNumber("Robot Input Voltage", pdh.getVoltage());
SmartDashboard.putNumber("PDH Temp", pdh.getTemperature());

Logger.recordOutput("Robot Total Current Amps", pdh.getTotalCurrent());
Logger.recordOutput("Robot Total Power Watts", pdh.getTotalPower());
Logger.recordOutput("Robot Total Energy Joules", pdh.getTotalEnergy());
Logger.recordOutput("Robot Input Voltage", pdh.getVoltage());
Logger.recordOutput("PDH Temp", pdh.getTemperature());
}

public void logPDHStickyFaults() {
SmartDashboard.putBoolean("PDH Channel 0", pdh.getStickyFaults().Channel0BreakerFault);
SmartDashboard.putBoolean("PDH Channel 1", pdh.getStickyFaults().Channel1BreakerFault);
SmartDashboard.putBoolean("PDH Channel 2", pdh.getStickyFaults().Channel2BreakerFault);
SmartDashboard.putBoolean("PDH Channel 3", pdh.getStickyFaults().Channel3BreakerFault);
SmartDashboard.putBoolean("PDH Channel 4", pdh.getStickyFaults().Channel4BreakerFault);
SmartDashboard.putBoolean("PDH Channel 5", pdh.getStickyFaults().Channel5BreakerFault);
SmartDashboard.putBoolean("PDH Channel 6", pdh.getStickyFaults().Channel6BreakerFault);
SmartDashboard.putBoolean("PDH Channel 7", pdh.getStickyFaults().Channel7BreakerFault);
SmartDashboard.putBoolean("PDH Channel 8", pdh.getStickyFaults().Channel8BreakerFault);
SmartDashboard.putBoolean("PDH Channel 9", pdh.getStickyFaults().Channel9BreakerFault);
SmartDashboard.putBoolean("PDH Channel 10", pdh.getStickyFaults().Channel10BreakerFault);
SmartDashboard.putBoolean("PDH Channel 11", pdh.getStickyFaults().Channel11BreakerFault);
SmartDashboard.putBoolean("PDH Channel 12", pdh.getStickyFaults().Channel12BreakerFault);
SmartDashboard.putBoolean("PDH Channel 13", pdh.getStickyFaults().Channel13BreakerFault);
SmartDashboard.putBoolean("PDH Channel 14", pdh.getStickyFaults().Channel14BreakerFault);
SmartDashboard.putBoolean("PDH Channel 15", pdh.getStickyFaults().Channel15BreakerFault);
SmartDashboard.putBoolean("PDH Channel 16", pdh.getStickyFaults().Channel16BreakerFault);
SmartDashboard.putBoolean("PDH Channel 17", pdh.getStickyFaults().Channel17BreakerFault);
SmartDashboard.putBoolean("PDH Channel 18", pdh.getStickyFaults().Channel18BreakerFault);
SmartDashboard.putBoolean("PDH Channel 19", pdh.getStickyFaults().Channel19BreakerFault);
SmartDashboard.putBoolean("PDH Channel 20", pdh.getStickyFaults().Channel20BreakerFault);
SmartDashboard.putBoolean("PDH Channel 21", pdh.getStickyFaults().Channel21BreakerFault);
SmartDashboard.putBoolean("PDH Channel 22", pdh.getStickyFaults().Channel22BreakerFault);
SmartDashboard.putBoolean("PDH Channel 23", pdh.getStickyFaults().Channel23BreakerFault);
SmartDashboard.putBoolean("PDH Channel Hardware Fault", pdh.getStickyFaults().HardwareFault);
SmartDashboard.putBoolean("PDH Channel Firmware Fault", pdh.getStickyFaults().FirmwareFault);
SmartDashboard.putBoolean("PDH Brownout", pdh.getStickyFaults().Brownout);
SmartDashboard.putBoolean("PDH CAN BUS Off", pdh.getStickyFaults().CanBusOff);
SmartDashboard.putBoolean("PDH Can Error", pdh.getStickyFaults().CanWarning);
SmartDashboard.putBoolean("PDH Has Reset", pdh.getStickyFaults().HasReset);



Logger.recordOutput("PDH Channel 0", pdh.getStickyFaults().Channel0BreakerFault);
Logger.recordOutput("PDH Channel 1", pdh.getStickyFaults().Channel1BreakerFault);
Logger.recordOutput("PDH Channel 2", pdh.getStickyFaults().Channel2BreakerFault);
Logger.recordOutput("PDH Channel 3", pdh.getStickyFaults().Channel3BreakerFault);
Logger.recordOutput("PDH Channel 4", pdh.getStickyFaults().Channel4BreakerFault);
Logger.recordOutput("PDH Channel 5", pdh.getStickyFaults().Channel5BreakerFault);
Logger.recordOutput("PDH Channel 6", pdh.getStickyFaults().Channel6BreakerFault);
Logger.recordOutput("PDH Channel 7", pdh.getStickyFaults().Channel7BreakerFault);
Logger.recordOutput("PDH Channel 8", pdh.getStickyFaults().Channel8BreakerFault);
Logger.recordOutput("PDH Channel 9", pdh.getStickyFaults().Channel9BreakerFault);
Logger.recordOutput("PDH Channel 10", pdh.getStickyFaults().Channel10BreakerFault);
Logger.recordOutput("PDH Channel 11", pdh.getStickyFaults().Channel11BreakerFault);
Logger.recordOutput("PDH Channel 12", pdh.getStickyFaults().Channel12BreakerFault);
Logger.recordOutput("PDH Channel 13", pdh.getStickyFaults().Channel13BreakerFault);
Logger.recordOutput("PDH Channel 14", pdh.getStickyFaults().Channel14BreakerFault);
Logger.recordOutput("PDH Channel 15", pdh.getStickyFaults().Channel15BreakerFault);
Logger.recordOutput("PDH Channel 16", pdh.getStickyFaults().Channel16BreakerFault);
Logger.recordOutput("PDH Channel 17", pdh.getStickyFaults().Channel17BreakerFault);
Logger.recordOutput("PDH Channel 18", pdh.getStickyFaults().Channel18BreakerFault);
Logger.recordOutput("PDH Channel 19", pdh.getStickyFaults().Channel19BreakerFault);
Logger.recordOutput("PDH Channel 20", pdh.getStickyFaults().Channel20BreakerFault);
Logger.recordOutput("PDH Channel 21", pdh.getStickyFaults().Channel21BreakerFault);
Logger.recordOutput("PDH Channel 22", pdh.getStickyFaults().Channel22BreakerFault);
Logger.recordOutput("PDH Channel 23", pdh.getStickyFaults().Channel23BreakerFault);
Logger.recordOutput("PDH Channel Hardward", pdh.getStickyFaults().HardwareFault);
Logger.recordOutput("PDH Channel Firmware Fault", pdh.getStickyFaults().FirmwareFault);
Logger.recordOutput("PDH Brownout", pdh.getStickyFaults().Brownout);
Logger.recordOutput("PDH CAN BUS Off", pdh.getStickyFaults().CanBusOff);
Logger.recordOutput("PDH Can Error", pdh.getStickyFaults().CanWarning);
Logger.recordOutput("PDH Has Reset", pdh.getStickyFaults().HasReset);

}
  private Swerve createSwerve() {
    return (Constants.COMPBOT) ? CompTunerConstants.createDrivetrain() : PracticeTunerConstants.createDrivetrain();
  }
}