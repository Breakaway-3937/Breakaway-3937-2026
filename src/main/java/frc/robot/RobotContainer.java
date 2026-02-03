package frc.robot;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shootdexer;
import frc.robot.subsystems.SuperSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

  //Driver Controllers
  private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
  private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());
  private final CommandXboxController xboxController = new CommandXboxController(
      Constants.Controllers.XBOX_CONTROLLER.getPort());

  //Subsystems
  private final Swerve s_Swerve = TunerConstants.createDrivetrain();
  private final Vision s_Vision = new Vision(s_Swerve);
  private final QuestNavSubsystem s_QuestNavSubsystem = new QuestNavSubsystem(s_Swerve,
      new Pose3d(3.418, 3.987, 0, new Rotation3d(0, 0, Math.toRadians(0121.115))));
  private final Shootdexer s_Shootdexer = new Shootdexer(s_Vision);
  private final Climber s_Climber = new Climber();
  private final Intake s_Intake = new Intake();
  private final SuperSubsystem s_SuperSubsystem = new SuperSubsystem(s_Shootdexer, s_Intake, s_Climber, s_Vision);

  //Misc
  private final SendableChooser<Command> autoChooser;
  private double translationMultiplier = 1.0;
  private double rotationMultiplier = 1.0;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.MAX_SPEED * Constants.Controllers.STICK_DEADBAND)
      .withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * Constants.Controllers.STICK_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  

  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Papa Smurf Jeffords", Commands.none());
    autoChooser.addOption("Arch Trench", new PathPlannerAuto("Arch Trench", false).withName("ArchTrench"));
    autoChooser.addOption("Arch Bump", new PathPlannerAuto("Arch Bump", false).withName("Arch Bump"));
    autoChooser.addOption("Trench 2 Trench", new PathPlannerAuto("Trench 2 Trench", false).withName("Trench 2 Trench"));
    SmartDashboard.putData("Auto Mode", autoChooser);

    s_Swerve.resetPose(new Pose2d(3.418, 3.987, new Rotation2d(121.115)));
    configureBindings();
  }

  private void configureBindings() {
    s_Swerve.setDefaultCommand(
        s_Swerve.applyRequest(() -> drive
            .withVelocityX(translationController.getX() * translationMultiplier * Constants.Swerve.MAX_SPEED)
            .withVelocityY(translationController.getY() * translationMultiplier * Constants.Swerve.MAX_SPEED)
            .withRotationalRate(rotationController.getX() * rotationMultiplier * Constants.Swerve.MAX_ANGULAR_RATE)));

    xboxController.y().onTrue(s_SuperSubsystem.autoTrack(true, true));
    xboxController.a().onTrue(s_SuperSubsystem.autoTrack(true, false));
    xboxController.b().onTrue(s_SuperSubsystem.autoTrack(false, null));
    xboxController.rightBumper().whileTrue(s_SuperSubsystem.fire());
  }

  public Command getAutonomousCommand() {
    SmartDashboard.putString("Current Auto", autoChooser.getSelected().getName());
    return autoChooser.getSelected();
  }
}
