package frc.robot.utility;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.generated.PracticeTunerConstants;

public final class Constants {

  public static class Controllers {
    public static final GenericHID TRANSLATION_CONTROLLER = new GenericHID(0);
    public static final GenericHID ROTATION_CONTROLLER = new GenericHID(1);
    public static final int TRANSLATION_BUTTON = 1;
    public static final GenericHID XBOX_CONTROLLER = new GenericHID(2);
    public static final double STICK_DEADBAND = 0.1;
  }

  public static class Swerve {
    public static final double MAX_SPEED = PracticeTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
  }

  public static final class Shootdexer {
    // Shooter
    public static final int SHOOTER_LEAD_CAN_ID = 15;
    public static final int SHOOTER_FOLLOW_CAN_ID = 16;
    public static final int HOOD_CAN_ID = 17;
    public static final int TURRET_CAN_ID = 12;
    // Indexer
    public static final int KICKER_CAN_ID = 13;
    public static final int DIVERTER_CAN_ID = 14;
    public static final int SPINNER_CAN_ID = 4;
    public static final int EYE_OF_SAURON_CAN_ID = 0;
  }

  public static final class Intake {
    public static final int INTAKE_CAN_ID = 5;
    public static final int INTAKE_WRIST_CAN_ID = 3;

  }

  public static final class Climber {
    public static final int CLIMBER_LEAD_CAN_ID = 0;
    public static final int CLIMBER_FOLLOW_CAN_ID = 0;
  }

  public static final class Vision {
    public static final String LEFT_CAMERA = "leftCamera";
    public static final String RIGHT_CAMERA = "rightCamera";

    public static final Transform3d LEFT_CAMERA_TRANSFORM = new Transform3d(
      new Translation3d(-0.2787156244, -0.2747080866, 0.219853637),
      new Rotation3d(0, -20, 137.726311));

    public static final Transform3d RIGHT_CAMERA_TRANSFORM = new Transform3d(
      new Translation3d(-0.2785432326, 0.274897723, 0.2198532637),
      new Rotation3d(0, -20, -137.726311));
  }

}
