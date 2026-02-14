package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.generated.TunerConstants;

public final class Constants {

  public static class Controllers {
    public static final GenericHID TRANSLATION_CONTROLLER = new GenericHID(0);
    public static final GenericHID ROTATION_CONTROLLER = new GenericHID(1);
    public static final GenericHID XBOX_CONTROLLER = new GenericHID(2);
    public static final double STICK_DEADBAND = 0.1;
  }

  public static class Swerve {
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
  }

  public static final class Shootdexer {
    // Shooter
    public static final int SHOOTER_LEAD_CAN_ID = 15;
    public static final int SHOOTER_FOLLOW_CAN_ID = 16;
    public static final int HOOD_CAN_ID = 0;
    public static final int TURRET_CAN_ID = 12;
    // Indexer
    public static final int KICKER_CAN_ID = 0;
    public static final int SPINER_CAN_ID = 4;
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

}
