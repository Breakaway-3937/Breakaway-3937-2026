package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoodexer extends SubsystemBase {
   private final TalonFX shooterLead, shooterFollow, hood, turretLead, turretFollow, kicker, spiner;
   private final CANrange kickerEntrance, kickerExit;

  public Shoodexer() {
    shooterLead = new TalonFX(Constants.Shoodexer.SHOOTER_LEAD_CAN_ID);
    shooterFollow = new TalonFX(Constants.Shoodexer.SHOOTER_FOLLOW_CAN_ID);
    hood = new TalonFX(Constants.Shoodexer.HOOD_CAN_ID);
    turretLead = new TalonFX(Constants.Shoodexer.TURRET_LEAD_CAN_ID);
    turretFollow = new TalonFX( Constants.Shoodexer.TURRET_FOLLOW_CAN_ID);
    kicker = new TalonFX(Constants.Shoodexer.KICKER_CAN_ID);
    spiner = new TalonFX(Constants.Shoodexer.SPINER_CAN_ID);
    kickerEntrance = new CANrange(Constants.Shoodexer.KICKER_ENTRANCE_CAN_ID);
    kickerExit = new CANrange(Constants.Shoodexer.KICKER_EXIT_CAN_ID);
  }

  public void configCANranges() {
    kickerEntrance.getConfigurator().apply(new CANrangeConfiguration());
    kickerExit.getConfigurator().apply(new CANrangeConfiguration());

    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.ProximityThreshold = 0.06;

    kickerEntrance.getConfigurator().apply(config);
    kickerExit.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
   
  }

}
/*
public static final int HOOD_CAN_ID = 0;
public static final int TURRET_LEAD_CAN_ID= 0;
public static final int TURRET_FOLLOW_CAN_ID= 0;
 */