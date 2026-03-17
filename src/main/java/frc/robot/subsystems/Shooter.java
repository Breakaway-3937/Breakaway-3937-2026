package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

public class Shooter extends SubsystemBase {

  private boolean isTracking = true;
  private double shooterSetter;
  private double hoodSetter;

  private final InterpolatingDoubleTreeMap hoodHubMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterHubMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodLobMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterLobMap = new InterpolatingDoubleTreeMap();

  private final TalonFX shooterLead, shooterFollow, hood, turret;
  private final CANrange eyeOfSauron;

  private final MotionMagicExpoVoltage turretRequest;
  private final MotionMagicExpoVoltage hoodRequest;
  private final VelocityTorqueCurrentFOC shooterRequest;

  private final Follower shooterFollowerRequest = new Follower(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID,
      MotorAlignmentValue.Opposed);

  private final double LOCKED_TURRET_ANGLE = 0.0, LOCKED_HOOD_ANGLE = 0.0;

  public static boolean isInHomeTerritory = false;

  public Shooter() {

    shooterLead = new TalonFX(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID);
    shooterFollow = new TalonFX(Constants.Shootdexer.SHOOTER_FOLLOW_CAN_ID);
    hood = new TalonFX(Constants.Shootdexer.HOOD_CAN_ID);
    turret = new TalonFX(Constants.Shootdexer.TURRET_CAN_ID);
    eyeOfSauron = new CANrange(Constants.Shootdexer.EYE_OF_SAURON_CAN_ID);

    turretRequest = new MotionMagicExpoVoltage(0);
    hoodRequest = new MotionMagicExpoVoltage(0);
    shooterRequest = new VelocityTorqueCurrentFOC(0);

    // 18.5
    hoodHubMap.put(5.6388, 5.79);
    // 17
    hoodHubMap.put(5.1816, 5.29);
    // 16
    hoodHubMap.put(4.8786, 4.36);
    // 15
    hoodHubMap.put(4.572, 4.22);
    // 14
    hoodHubMap.put(4.2672, 4.43);
    // 13
    hoodHubMap.put(3.9624, 3.79);
    // 12
    hoodHubMap.put(3.6576, 3.79);
    // 11
    hoodHubMap.put(3.3528, 2.9);
    // 10
    hoodHubMap.put(3.048, 2.82);
    // 9
    hoodHubMap.put(2.7432, 2.81);
    // 8
    hoodHubMap.put(2.4384, 2.17);
    // 7
    hoodHubMap.put(2.1336, 1.60);
    // 6
    hoodHubMap.put(1.8288, 1.60);

    // I added 1 RPS to the points
    // 18.5
    shooterHubMap.put(5.6388, 3260.0);
    // 17
    shooterHubMap.put(5.1816, 3160.0);
    // 16
    shooterHubMap.put(4.8786, 3160.0);
    // 15
    shooterHubMap.put(4.572, 3060.0);
    // 14
    shooterHubMap.put(4.2672, 2910.0);
    // 13
    shooterHubMap.put(3.9624, 2810.0);
    // 12
    shooterHubMap.put(3.6576, 2760.0);
    // 11
    shooterHubMap.put(3.3528, 2660.0);
    // 10
    shooterHubMap.put(3.048, 2560.0);
    // 9
    shooterHubMap.put(2.7432, 2410.0);
    // 8
    shooterHubMap.put(2.4384, 2310.0);
    // 7
    shooterHubMap.put(2.1336, 2310.0);
    // 6
    shooterHubMap.put(1.8288, 2060.0);

    hoodLobMap.put(3.7846, 3.84);
    hoodLobMap.put(5.1816, 4.92);
    hoodLobMap.put(6.096, 5.98);
    hoodLobMap.put(7.0104, 6.09);
    hoodLobMap.put(8.2296, 6.27);
    hoodLobMap.put(9.7536, 7.6);
    hoodLobMap.put(12.4968, 8.1);

    shooterLobMap.put(3.7846, 2500.0);
    shooterLobMap.put(5.1816, 2700.0);
    shooterLobMap.put(6.096, 2850.0);
    shooterLobMap.put(7.0104, 3100.0);
    shooterLobMap.put(8.2296, 3500.0);
    shooterLobMap.put(9.7536, 3900.0);
    shooterLobMap.put(12.4968, 4600.0);

    configTurret();
    configHood();
    configShooter();
    configCANranges();

    SmartDashboard.putNumber("Shooter Added Speed", 0);
    SmartDashboard.putNumber("Hood Added Angle", 0);
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

    config.MotionMagic.MotionMagicExpo_kV = 0.01;
    config.MotionMagic.MotionMagicExpo_kA = 0.05;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    turret.getConfigurator().apply(config);
    turret.setPosition(0);
  }

  public void configHood() {

    hood.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.2;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 6.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicExpo_kV = 0.01;
    config.MotionMagic.MotionMagicExpo_kA = 0.05;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    hood.getConfigurator().apply(config);
    hood.setPosition(0);
  }

  public void configShooter() {

    shooterLead.getConfigurator().apply(new TalonFXConfiguration());
    shooterFollow.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kP = 5.742;
    config.Slot0.kS = 9.0;
    config.Slot0.kV = 0.143;

    // config.Slot0.kP = 6.0;
    // config.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    // config.TorqueCurrent.PeakReverseTorqueCurrent = -60;
    // config.MotorOutput.PeakForwardDutyCycle = 1;
    // config.MotorOutput.PeakReverseDutyCycle = -1;

    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.CurrentLimits.SupplyCurrentLimit = 40;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 75;

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

  public void setAutoTracking(boolean isTracking) {
    this.isTracking = isTracking;
  }

  public Command runShooter() {

    return runOnce(() -> {

      double velocity;
      double currentDistance = Vision.getAdjustedDistance();

      if (isTracking) {

        if (isInHomeTerritory) {

          velocity = shooterHubMap.get(currentDistance) / 60.0 + shooterSetter;

        } else {

          velocity = shooterLobMap.get(currentDistance) / 60.0 + shooterSetter;

        }
      } else {
      
        velocity = 33.3;

      }

      shooterLead.setControl(shooterRequest.withVelocity(velocity));
    });

  }

  public Command idleShooter() {
    return runOnce(() -> shooterLead.setControl(shooterRequest.withVelocity(10)));
  }

  public Command stopShooter() {
    return runOnce(() -> shooterLead.setControl(shooterRequest.withVelocity(0)));
  }

  public BooleanSupplier isAtSpeed() {
    if (isTracking) {
      return () -> shooterLead.getVelocity()
          .getValueAsDouble() > (shooterHubMap.get(Vision.getAdjustedDistance()) / 60.0 + shooterSetter) - 0.8;
    } else {
      return () -> shooterLead.getVelocity().getValueAsDouble() > (33.3) - 0.8;
    }
  }

  public double getTurretPosition() {
    return turret.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    isInHomeTerritory = Vision.isInHomeTerritory();
    if (isTracking) {

      turret.setControl(turretRequest.withPosition(Vision.getAdjustedTurretAngle()));

      if (!Vision.isUnderTrench()) {
        if (Vision.isInHomeTerritory()) {
          hood.setControl(hoodRequest.withPosition(hoodHubMap.get(Vision.getAdjustedDistance()) + hoodSetter));
        } else {
          hood.setControl(hoodRequest.withPosition(hoodLobMap.get(Vision.getAdjustedDistance()) + hoodSetter));
        }
      } else {
        hood.setControl(hoodRequest.withPosition(LOCKED_HOOD_ANGLE));
      }
    } else {
      hood.setControl(hoodRequest.withPosition(LOCKED_HOOD_ANGLE));
      turret.setControl(turretRequest.withPosition(LOCKED_TURRET_ANGLE));
    }

    SmartDashboard.putNumber("Shooter Speed RPS", shooterLead.getVelocity().getValueAsDouble() * 60.0);
    SmartDashboard.putNumber("Shooter Setpoint", shooterHubMap.get(Vision.getAdjustedDistance()));
    shooterSetter = SmartDashboard.getNumber("Shooter Added Speed", 1400);
    hoodSetter = SmartDashboard.getNumber("Hood Added Angle", 1400);
  }

}