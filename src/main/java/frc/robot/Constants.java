package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

public final class Constants {

  public static class Controllers {
    public static final int DRIVE_CONTROLLER = 0;
    public static final double STICK_DEADBAND = 0.1;
  }

  public static class Swerve {
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
  }

    public static final class Shoodexer{
public static final int SHOOTER_LEAD_CAN_ID = 0;
public static final int SHOOTER_FOLLOW_CAN_ID = 0;
public static final int HOOD_CAN_ID = 0;
public static final int TURRET_LEAD_CAN_ID= 0;
public static final int TURRET_FOLLOW_CAN_ID= 0;
//INDEXER
public static final int KICKER_CAN_ID = 0;
public static final int SPINER_CAN_ID = 0;
public static final int KICKER_ENTRANCE_CAN_ID = 0;
public static final int KICKER_EXIT_CAN_ID = 0;
  }
    public static final class Intake{
public static final int INTAKE_CAN_ID = 0;
public static final int INTAKE_POSITION_CAN_ID = 0;

  }
    public static final class Climber{
public static final int OUTER_CLIMBER_CAN_ID = 0;
public static final int INNER_CLIMBER_CAN_ID = 0;
public static final int EYE_OF_SAURON_CAN_ID = 0;
  }

}

