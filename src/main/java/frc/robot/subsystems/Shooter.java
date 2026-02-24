package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.conduit.schema.PDPData;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Calculations;
import frc.robot.utility.Constants;

public class Shooter extends SubsystemBase {
  private final Calculations s_Calculations;

  private boolean isTracking = true;
  private boolean isHub = true;

  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

  private final TalonFX shooterLead, shooterFollow, hood, turret;
  private final CANrange eyeOfSauron;

  private final MotionMagicExpoVoltage turretRequest;
  private final MotionMagicExpoVoltage hoodRequest;
  private final MotionMagicVelocityVoltage shooterRequest;

  private final Follower shooterFollowerRequest = new Follower(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID,
      MotorAlignmentValue.Opposed);

  private final double LOCKED_TURRET_ANGLE = 0.0, LOCKED_HOOD_ANGLE = 0.0;

  public Shooter(Calculations s_Calculations) {
    this.s_Calculations = s_Calculations;

    shooterLead = new TalonFX(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID);
    shooterFollow = new TalonFX(Constants.Shootdexer.SHOOTER_FOLLOW_CAN_ID);
    hood = new TalonFX(Constants.Shootdexer.HOOD_CAN_ID);
    turret = new TalonFX(Constants.Shootdexer.TURRET_CAN_ID);
    eyeOfSauron = new CANrange(Constants.Shootdexer.EYE_OF_SAURON_CAN_ID);

    turretRequest = new MotionMagicExpoVoltage(0);
    hoodRequest = new MotionMagicExpoVoltage(0);
    shooterRequest = new MotionMagicVelocityVoltage(0);

    hoodMap.put(1.524, 0.0);
    hoodMap.put(25.908, 4.5);
    hoodMap.put(4.572, 4.3);
    hoodMap.put(3.6576, 3.4);
    hoodMap.put(3.048, 3.0);
    hoodMap.put(2.1336, 1.55);

    shooterMap.put(1.524, 39.2);
    shooterMap.put(5.1816, 51.25);
    shooterMap.put(4.572, 48.75);
    shooterMap.put(3.6576, 45.5);
    shooterMap.put(3.048, 43.5);
    shooterMap.put(2.1336, 41.5);

    configTurret();
    configHood();
    configShooter();
    configCANranges();

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

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;
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
    config.Slot0.kV = 0.14;
    config.Slot0.kA = 0.01;
    config.Slot0.kP = 0.01;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicExpo_kV = 0.3;
    config.MotionMagic.MotionMagicExpo_kA = 0.1;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;
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
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.00;
    config.Slot0.kP = 0.07;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 900;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    shooterLead.getConfigurator().apply(config);
    shooterFollow.getConfigurator().apply(config);
    shooterFollow.setControl(shooterFollowerRequest);
  }

  public void configCANranges() {
    eyeOfSauron.getConfigurator().apply(new CANrangeConfiguration());

    CANrangeConfiguration config = new CANrangeConfiguration();

    config.ProximityParams.ProximityThreshold = 0.06;

    eyeOfSauron.getConfigurator().apply(config);
  }

  public void setAutoTracking(boolean isTracking, boolean isHub) {
    this.isTracking = isTracking;
    this.isHub = isHub;
  }

  public Command runShooter() {
    return runOnce(() -> shooterLead.setControl(shooterRequest.withVelocity(shooterMap.get(s_Calculations.getAdjustedDistance()))));
  }

  public Command idleShooter() {
    return runOnce(() -> shooterLead.setControl(shooterRequest.withVelocity(10)));
  }

  public BooleanSupplier isAtSpeed() {
    return () -> shooterLead.getVelocity().getValueAsDouble() > shooterMap.get(s_Calculations.getAdjustedDistance()) - 3;
  }

  @Override
  public void periodic() {
    s_Calculations.runCalculations();

    s_Calculations.setTarget(isHub);

    if (isTracking && !s_Calculations.isUnderTrench()) {
      hood.setControl(hoodRequest.withPosition(hoodMap.get(s_Calculations.getAdjustedDistance())));
      turret.setControl(turretRequest.withPosition(s_Calculations.getAdjustedTurretAngle()));
    } else if (isTracking && s_Calculations.isUnderTrench()) {
      hood.setControl(hoodRequest.withPosition(LOCKED_HOOD_ANGLE));
      turret.setControl(turretRequest.withPosition(s_Calculations.getAdjustedTurretAngle()));
    } else {
      hood.setControl(hoodRequest.withPosition(s_Calculations.getAdjustedDistance()));
      turret.setControl(turretRequest.withPosition(LOCKED_TURRET_ANGLE));
    }

    SmartDashboard.putNumber("Hood Angle", hood.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Speed RPS", shooterLead.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Setpoint", shooterMap.get(s_Calculations.getAdjustedDistance()));
    SmartDashboard.putNumber("Turret Angle", s_Calculations.getAdjustedTurretAngle());
    SmartDashboard.putBoolean("Shooter Up To Speed", shooterLead.getVelocity().getValueAsDouble() > shooterMap.get(s_Calculations.getAdjustedDistance()) - 3);
  }

}