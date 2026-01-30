package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shootdexer extends SubsystemBase {
  private final Vision s_Vision;
  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private final PIDController hoodPID = new PIDController(0.1, 0.0, 0.0);
  private final InterpolatingDoubleTreeMap turretMap = new InterpolatingDoubleTreeMap();
  private final PIDController turretPID = new PIDController(0.1, 0.0, 0.0);
  private final TalonFX shooterLead, shooterFollow, hood, turretLead, turretFollow, kicker, spiner;
  private final CANrange kickerEntrance, kickerExit;
  private final Follower shooterFollowerRequest = new Follower(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID, null);
  private final Follower turretFollowerRequest = new Follower(Constants.Shootdexer.TURRET_LEAD_CAN_ID, null);
  private boolean autoTracking = false;

  public Shootdexer(Vision s_Vision) {
    this.s_Vision = s_Vision;
    shooterLead = new TalonFX(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID);
    shooterFollow = new TalonFX(Constants.Shootdexer.SHOOTER_FOLLOW_CAN_ID);
    hood = new TalonFX(Constants.Shootdexer.HOOD_CAN_ID);
    turretLead = new TalonFX(Constants.Shootdexer.TURRET_LEAD_CAN_ID);
    turretFollow = new TalonFX(Constants.Shootdexer.TURRET_FOLLOW_CAN_ID);
    kicker = new TalonFX(Constants.Shootdexer.KICKER_CAN_ID);
    spiner = new TalonFX(Constants.Shootdexer.SPINER_CAN_ID);
    kickerEntrance = new CANrange(Constants.Shootdexer.KICKER_ENTRANCE_CAN_ID);
    kickerExit = new CANrange(Constants.Shootdexer.KICKER_EXIT_CAN_ID);

    hoodMap.put(1.0, 2.0);

    turretMap.put(1.0, 2.0);

    configMotors();
    configCANranges();
  }

  public void configMotors() {
    shooterFollow.setControl(shooterFollowerRequest);
    turretFollow.setControl(turretFollowerRequest);
    turretPID.enableContinuousInput(-180.0, 180.0);
  }

  public void configCANranges() {
    kickerEntrance.getConfigurator().apply(new CANrangeConfiguration());
    kickerExit.getConfigurator().apply(new CANrangeConfiguration());

    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.ProximityThreshold = 0.06;

    kickerEntrance.getConfigurator().apply(config);
    kickerExit.getConfigurator().apply(config);
  }

  public void setAutoTracking(boolean autoTracking) {
    if(autoTracking) {
      hoodPID.setSetpoint(hoodMap.get(s_Vision.getDistance()));
      turretPID.setSetpoint(turretMap.get(s_Vision.getDistance()));
    }
    else {
      hoodPID.setSetpoint(States.ShootdexerStates.LOCKED_IDLE.getHoodAngle());
      turretPID.setSetpoint(States.ShootdexerStates.LOCKED_IDLE.getTurretRotation());
    }
  }

  @Override
  public void periodic() {
    
  }

}