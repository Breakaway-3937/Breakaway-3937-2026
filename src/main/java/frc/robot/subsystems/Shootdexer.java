package frc.robot.subsystems;

import java.lang.Thread.State;
import java.lang.constant.DirectMethodHandleDesc;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Calculations;
import frc.robot.subsystems.States.IntakeStates;
import frc.robot.subsystems.States.ShootdexerStates;

public class Shootdexer extends SubsystemBase {
  private final Calculations s_Vision;

  private boolean isTracking = true;

  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  private final TalonFX shooterLead, shooterFollow, hood, turret, kicker, spinner, diverter;
  private final CANrange eyeOfSauron;

  private final MotionMagicExpoVoltage turretRequest;
  private final MotionMagicExpoVoltage hoodRequest;
  private final MotionMagicVelocityVoltage shooterRequest;
  private final MotionMagicVelocityVoltage spinnerRequest;
  private final MotionMagicVelocityVoltage kickerRequest;
  private final MotionMagicVelocityVoltage diverterRequest;

  private final Follower shooterFollowerRequest = new Follower(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID,
      MotorAlignmentValue.Opposed);

  private ShootdexerStates shootdexerState = ShootdexerStates.IDLE;

  private final double LOCKED_TURRET_ANGLE = 0.0, LOCKED_HOOD_ANGLE = 0.0;

  public Shootdexer(Calculations s_Vision) {
    this.s_Vision = s_Vision;

    shooterLead = new TalonFX(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID);
    shooterFollow = new TalonFX(Constants.Shootdexer.SHOOTER_FOLLOW_CAN_ID);
    hood = new TalonFX(Constants.Shootdexer.HOOD_CAN_ID);
    turret = new TalonFX(Constants.Shootdexer.TURRET_CAN_ID);
    kicker = new TalonFX(Constants.Shootdexer.KICKER_CAN_ID);
    diverter = new TalonFX(Constants.Shootdexer.DIVERTER_CAN_ID);
    spinner = new TalonFX(Constants.Shootdexer.SPINNER_CAN_ID);
    eyeOfSauron = new CANrange(Constants.Shootdexer.EYE_OF_SAURON_CAN_ID);

    turretRequest = new MotionMagicExpoVoltage(0);
    hoodRequest = new MotionMagicExpoVoltage(0);
    spinnerRequest = new MotionMagicVelocityVoltage(0);
    kickerRequest = new MotionMagicVelocityVoltage(0);
    diverterRequest = new MotionMagicVelocityVoltage(0);
    shooterRequest = new MotionMagicVelocityVoltage(0);

    hoodMap.put(1.0, 2.0);

    configTurret();
    configHood();
    configShooter();
    configCANranges();
    configSpinner();
    configDiverter();
    configKicker();
  }

  public void configTurret() {

    turret.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.2;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 6.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.Slot1.kS = 0.0;
    config.Slot1.kV = 0.12;
    config.Slot1.kA = 0.0;
    config.Slot1.kP = 20.0;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = 0.0;

    config.MotionMagic.MotionMagicExpo_kV = 0.005;
    config.MotionMagic.MotionMagicExpo_kA = 0.1;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    turret.getConfigurator().apply(config);
    turret.setPosition(0);
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

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.00;
    config.Slot0.kP = 0.07;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 900;

    shooterLead.getConfigurator().apply(config);
    shooterFollow.getConfigurator().apply(config);
    shooterFollow.setControl(shooterFollowerRequest);
  }

  public void configSpinner() {

    spinner.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.07;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 900;

    spinner.getConfigurator().apply(config);
  }

  public void configKicker() {
    kicker.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.07;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 900;

    kicker.getConfigurator().apply(config);
  }

  public void configDiverter() {
    diverter.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.07;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 900;

    diverter.getConfigurator().apply(config);
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
    return runOnce(() -> shooterLead.setControl(shooterRequest.withVelocity(-1000)));
  }

  public Command idleShooter() {
    return runOnce(() -> shooterLead.setControl(shooterRequest.withVelocity(-20)));
  }

  public void setShootdexerState(ShootdexerStates shootdexerState) {
    this.shootdexerState = shootdexerState;
  }

  public void setShootdexerRequests() {
    kicker.setControl(kickerRequest.withVelocity(shootdexerState.getKickerSpeed()));
    diverter.setControl(diverterRequest.withVelocity(shootdexerState.getKickerSpeed()));
    spinner.setControl(spinnerRequest.withVelocity(shootdexerState.getSpinnerSpeed()));
    shooterLead.setControl(shooterRequest.withVelocity(shootdexerState.getShooterSpeed()));
  }

  public Command setShootdexer() {
    return runOnce(() -> setShootdexerRequests());
  }

  public void math() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putString("ShootdexerState", shootdexerState.toString());
    SmartDashboard.putNumber("Turret Motor", turret.getPosition().getValueAsDouble());
    if (isTracking && !s_Vision.isUnderTrench()) {
      // hood.setControl(hoodRequest.withPosition(hoodMap.get(s_Vision.getDistanceToTarget())));
      s_Vision.setTurretAngle(turret, turretRequest);
    } else if (isTracking && s_Vision.isUnderTrench()) {
      // hood.setControl(hoodRequest.withPosition(LOCKED_HOOD_ANGLE));
      s_Vision.setTurretAngle(turret, turretRequest);
    } else {
      // hood.setControl(hoodRequest.withPosition(LOCKED_HOOD_ANGLE));
      turret.setControl(turretRequest.withPosition(LOCKED_TURRET_ANGLE).withSlot(0));
    }
  }

}

// This is a shoodexter