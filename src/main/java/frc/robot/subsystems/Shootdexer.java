package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shootdexer extends SubsystemBase {
  private final Vision s_Vision;
  private boolean isTracking = true;
  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private final MotionMagicExpoVoltage hoodRequest;
  private final InterpolatingDoubleTreeMap turretMap = new InterpolatingDoubleTreeMap();
  private final MotionMagicExpoVoltage turretRequest;
  private final TalonFX shooterLead, shooterFollow, hood, turretLead, turretFollow, kicker, spiner;
  private final Follower shooterFollowerRequest = new Follower(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID, null);
  private final Follower turretFollowerRequest = new Follower(Constants.Shootdexer.TURRET_LEAD_CAN_ID, null);
  private final CANrange kickerEntrance, kickerExit;

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

    turretMap.put(1.0, 8.5);

    configTurret();
    configHood();
    configShooter();
    configCANranges();

    hoodRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);
    turretRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);
  }

  public void configTurret() {

    turretLead.getConfigurator().apply(new TalonFXConfiguration());
    turretFollow.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kG = 0.0;

    config.MotionMagic.MotionMagicExpo_kV = 0.0;
    config.MotionMagic.MotionMagicExpo_kA = 0.0;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    turretLead.getConfigurator().apply(config);
    turretFollow.getConfigurator().apply(config);
    turretLead.setPosition(0);
    turretFollow.setPosition(0);
    turretFollow.setControl(turretFollowerRequest);
  }

  public void configHood() {

    hood.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kG = 0.0;

    config.MotionMagic.MotionMagicExpo_kV = 0.0;
    config.MotionMagic.MotionMagicExpo_kA = 0.0;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    hood.getConfigurator().apply(config);
    hood.setPosition(0);
  }

  public void configShooter() {

    shooterLead.getConfigurator().apply(new TalonFXConfiguration());
    shooterFollow.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    shooterLead.getConfigurator().apply(config);
    shooterFollow.getConfigurator().apply(config);
  }

  public void configCANranges() {
    kickerEntrance.getConfigurator().apply(new CANrangeConfiguration());
    kickerExit.getConfigurator().apply(new CANrangeConfiguration());

    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.ProximityThreshold = 0.06;

    kickerEntrance.getConfigurator().apply(config);
    kickerExit.getConfigurator().apply(config);
  }

  public void setAutoTracking(boolean isTracking, Boolean isHub) {
    if (isHub != null) {
      s_Vision.setTarget(isHub.booleanValue());
    }
    this.isTracking = isTracking;
  }
  
  public Command runShooter() {
    return runOnce(() -> shooterLead.set(SmartDashboard.getNumber("Shooter Speed", 0)));
  }

  public Command setSpinerForward() {
    return runOnce(() -> spiner.set(0.1));
  }

  public Command setSpinerBackward() {
    return runOnce(() -> spiner.set(-0.1));
  }

  public Command stopSpiner() {
    return runOnce(() -> spiner.set(0.0));
  }

  public Command setKickerForward() {
    return runOnce(() -> spiner.set(0.1));
  }

  public Command setKickerBackward() {
    return runOnce(() -> spiner.set(-0.1));
  }

  public Command stopKicker() {
    return runOnce(() -> spiner.set(0.0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", 0);
    if (isTracking) {
      hood.setControl(hoodRequest.withPosition(hoodMap.get(s_Vision.getDistance())));
      turretLead.setControl(turretRequest.withPosition(turretMap.get(s_Vision.getAngle())));
    } else {
      hood.setControl(hoodRequest.withPosition(States.ShootdexerStates.LOCKED_IDLE.getHoodAngle()));
      turretLead.setControl(turretRequest.withPosition(States.ShootdexerStates.LOCKED_IDLE.getTurretRotation()));
    }
  }

}

// This is a shoodexter