package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shootdexer extends SubsystemBase {
   private final TalonFX shooterLead, shooterFollow, hood, turretLead, turretFollow, kicker, spiner;
   private final CANrange kickerEntrance, kickerExit;
   private final Follower shooterFollowerRequest = new Follower(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID, null);
   private final Follower turretFollowerRequest = new Follower(Constants.Shootdexer.TURRET_LEAD_CAN_ID, null);

  public Shootdexer() {
    shooterLead = new TalonFX(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID);
    shooterFollow = new TalonFX(Constants.Shootdexer.SHOOTER_FOLLOW_CAN_ID);
    hood = new TalonFX(Constants.Shootdexer.HOOD_CAN_ID);
    turretLead = new TalonFX(Constants.Shootdexer.TURRET_LEAD_CAN_ID);
    turretFollow = new TalonFX( Constants.Shootdexer.TURRET_FOLLOW_CAN_ID);
    kicker = new TalonFX(Constants.Shootdexer.KICKER_CAN_ID);
    spiner = new TalonFX(Constants.Shootdexer.SPINER_CAN_ID);
    kickerEntrance = new CANrange(Constants.Shootdexer.KICKER_ENTRANCE_CAN_ID);
    kickerExit = new CANrange(Constants.Shootdexer.KICKER_EXIT_CAN_ID);

    configMotors();
    configCANranges();
  }

  public void configMotors() {
    shooterFollow.setControl(shooterFollowerRequest);
    turretFollow.setControl(turretFollowerRequest);
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