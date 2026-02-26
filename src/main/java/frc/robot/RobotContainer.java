package frc.robot;

import frc.robot.subsystems.Swerve;
import frc.robot.utility.Calculations;
import frc.robot.utility.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SuperSubsystem;
import frc.robot.generated.PracticeTunerConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

  // Driver Controllers
  private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
  private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());
  private final CommandXboxController xboxController = new CommandXboxController(
      Constants.Controllers.XBOX_CONTROLLER.getPort());

  // Subsystems
  private final Swerve s_Swerve = PracticeTunerConstants.createDrivetrain();
  //private final QuestNavSubsystem s_QuestNavSubsystem2
  private final QuestNavSubsystem s_QuestNavSubsystem = new QuestNavSubsystem(s_Swerve);
  private final Calculations s_Calculations = new Calculations(s_Swerve, s_QuestNavSubsystem);
  private final Shooter s_Shooter = new Shooter(s_Calculations);
  // private final Climber s_Climber = new Climber();
  private final Indexer s_Indexer = new Indexer();
  private final Intake s_Intake = new Intake();
  private final SuperSubsystem s_SuperSubsystem = new SuperSubsystem(s_Shooter, s_Indexer, s_Intake /* s_Climber */);

  // Misc
  private final SendableChooser<Command> autoChooser;
  private double translationMultiplier = 1.0;
  private double rotationMultiplier = 1.0;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.MAX_SPEED * Constants.Controllers.STICK_DEADBAND)
      .withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * Constants.Controllers.STICK_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public RobotContainer() {

    // NamedCommands.registerCommand("Shoot", s_SuperSubsystem.fire());
    // NamedCommands.registerCommand("Intake", s_SuperSubsystem.intake());
    // NamedCommands.registerCommand("Climb", s_SuperSubsystem.climbRungOne());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Papa Smurf Jeffords", Commands.none());
    autoChooser.addOption("Arch Trench", new PathPlannerAuto("Arch Trench", false).withName("ArchTrench"));
    autoChooser.addOption("Arch Bump", new PathPlannerAuto("Arch Bump", false).withName("Arch Bump"));
    autoChooser.addOption("Trench 2 Trench", new PathPlannerAuto("Trench 2 Trench", false).withName("Trench 2 Trench"));
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

    xboxController.y().onTrue(s_SuperSubsystem.autoTrack(true, true));
    xboxController.a().onTrue(s_SuperSubsystem.autoTrack(true, false));
    xboxController.b().onTrue(s_SuperSubsystem.autoTrack(false, true));
    xboxController.leftBumper().onTrue(s_SuperSubsystem.unclog()).onFalse(s_SuperSubsystem.stopIntake());
    xboxController.leftTrigger(0.3).and(xboxController.rightTrigger(0.3)).onTrue(s_SuperSubsystem.combo());
    xboxController.leftTrigger(0.3).onTrue(s_SuperSubsystem.intake()).onFalse(s_SuperSubsystem.stopIntake());
    xboxController.rightTrigger(0.3).onTrue(s_SuperSubsystem.fire().repeatedly()).onFalse(s_SuperSubsystem.idle());
    xboxController.rightStick().onTrue(s_SuperSubsystem.protectIntake());
  }

  public Command getAutonomousCommand() {
    SmartDashboard.putString("Current Auto", autoChooser.getSelected().getName());
    return autoChooser.getSelected();
  }

  public Swerve getSwerve() {
    return s_Swerve;
  }

  public QuestNavSubsystem getQuestNavSubsystem() {
    return s_QuestNavSubsystem;
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

  /*
   * public Climber getClimber() {
   * return s_Climber;
   * }
   */
}
