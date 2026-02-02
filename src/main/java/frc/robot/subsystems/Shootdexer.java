package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.States.ShootdexerStates;

public class Shootdexer extends SubsystemBase {
  private final Vision s_Vision;
  private boolean isTracking = true;
  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private final MotionMagicExpoVoltage hoodRequest;
  private final InterpolatingDoubleTreeMap turretMap = new InterpolatingDoubleTreeMap();
  private final MotionMagicExpoVoltage turretRequest;
  private final TalonFX shooterLead, shooterFollow, hood, turretLead, turretFollow, kicker, spiner;
  private final Follower shooterFollowerRequest = new Follower(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID,
      MotorAlignmentValue.Aligned);
  private final Follower turretFollowerRequest = new Follower(Constants.Shootdexer.TURRET_LEAD_CAN_ID,
      MotorAlignmentValue.Aligned);
  private final CANrange eyeOfSauron;
  private ShootdexerStates shootdexerState = ShootdexerStates.IDLE;
  private final MotionMagicVoltage spinerRequest;
  private final MotionMagicVoltage kickerRequest;
  private final double LOCKED_TURRET_ANGLE = 0.0, LOCKED_HOOD_ANGLE = 0.0;

  public Shootdexer(Vision s_Vision) {
    this.s_Vision = s_Vision;
    shooterLead = new TalonFX(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID);
    shooterFollow = new TalonFX(Constants.Shootdexer.SHOOTER_FOLLOW_CAN_ID);
    hood = new TalonFX(Constants.Shootdexer.HOOD_CAN_ID);
    turretLead = new TalonFX(Constants.Shootdexer.TURRET_LEAD_CAN_ID);
    turretFollow = new TalonFX(Constants.Shootdexer.TURRET_FOLLOW_CAN_ID);
    kicker = new TalonFX(Constants.Shootdexer.KICKER_CAN_ID);
    spiner = new TalonFX(Constants.Shootdexer.SPINER_CAN_ID);

    eyeOfSauron = new CANrange(Constants.Shootdexer.EYE_OF_SAURON_CAN_ID);

    hoodMap.put(1.0, 2.0);

    turretMap.put(1.0, 8.5);

    configTurret();
    configHood();
    configShooter();
    configCANranges();

    hoodRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);
    turretRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);

    spinerRequest = new MotionMagicVoltage(null);
    kickerRequest = new MotionMagicVoltage(null);
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

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    shooterLead.getConfigurator().apply(config);
    shooterFollow.getConfigurator().apply(config);
    shooterLead.setPosition(0);
    shooterFollow.setPosition(0);
    shooterFollow.setControl(shooterFollowerRequest);
  }

  public void configSpiner() {

    spiner.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    spiner.getConfigurator().apply(config);
  }

  public void configKicker() {
    kicker.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    kicker.getConfigurator().apply(config);
  }

  public void configCANranges() {
    eyeOfSauron.getConfigurator().apply(new CANrangeConfiguration());

    CANrangeConfiguration config = new CANrangeConfiguration();

    config.ProximityParams.ProximityThreshold = 0.06;

    eyeOfSauron.getConfigurator().apply(config);
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

  public void setShootDexerState(ShootdexerStates shootdexerState) {
    this.shootdexerState = shootdexerState;
  }

  public Command setShooterPower() {
    return runOnce(() -> shooterLead.set(shootdexerState.getShooterSpeed()));
  }

  public Command setSpiner() {
    return runOnce(() -> spiner.setControl(spinerRequest.withPosition(shootdexerState.getSpinnerSpeed())));
  }

  public Command setKicker() {
    return runOnce(() -> kicker.setControl(kickerRequest.withPosition(shootdexerState.getKickerSpeed())));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", 0);
    if (isTracking && !s_Vision.isUnderTrench()) {
      hood.setControl(hoodRequest.withPosition(hoodMap.get(s_Vision.getDistanceToTarget())));
      turretLead.setControl(turretRequest.withPosition(turretMap.get(s_Vision.getAngleToTarget())));
    } else if (isTracking && s_Vision.isUnderTrench()) {
      hood.setControl(hoodRequest.withPosition(LOCKED_HOOD_ANGLE));
      turretLead.setControl(turretRequest.withPosition(turretMap.get(s_Vision.getAngleToTarget())));
    } else {
      hood.setControl(hoodRequest.withPosition(LOCKED_HOOD_ANGLE));
      turretLead.setControl(turretRequest.withPosition(LOCKED_TURRET_ANGLE));
    }
  }

}

// This is a shoodexter