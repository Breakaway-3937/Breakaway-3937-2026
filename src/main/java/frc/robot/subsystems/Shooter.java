package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
    private static final int shootLeadPDH = 17;
  private static final int shootFollowPDH = 13;
  private static final int turretPDH = 12;
  private static final int hoodPDH = 15;
  private final PowerDistribution pdh;
  //private final PowerDistribution pdh = new PowerDistribution(27, ModuleType.kRev);

  public Shooter(PowerDistribution pdh) {
    this.pdh = pdh;

    shooterLead = new TalonFX(Constants.Shootdexer.SHOOTER_LEAD_CAN_ID);
    shooterFollow = new TalonFX(Constants.Shootdexer.SHOOTER_FOLLOW_CAN_ID);
    hood = new TalonFX(Constants.Shootdexer.HOOD_CAN_ID);
    turret = new TalonFX(Constants.Shootdexer.TURRET_CAN_ID);
    eyeOfSauron = new CANrange(Constants.Shootdexer.EYE_OF_SAURON_CAN_ID);

    turretRequest = new MotionMagicExpoVoltage(0);
    hoodRequest = new MotionMagicExpoVoltage(0);
    shooterRequest = new VelocityTorqueCurrentFOC(0);

    hoodHubMap.put(7.9248, 7.68);
    
    hoodHubMap.put(7.3152, 7.68);
    
    hoodHubMap.put(6.7056, 7.68);

    hoodHubMap.put(6.013, 6.82);
    
    hoodHubMap.put(5.412, 6.32);
    
    hoodHubMap.put(4.852, 5.32);

    hoodHubMap.put(4.25, 4.82);
    // 12
    hoodHubMap.put(3.6576, 4.29);
    // 11
    hoodHubMap.put(3.3528, 3.4);
    // 10
    hoodHubMap.put(3.048, 3.32);
    // 9
    hoodHubMap.put(2.7432, 3.31);
    // 8
    hoodHubMap.put(2.4384, 2.67);
    // 7
    hoodHubMap.put(2.1336, 2.10);
    // 6
    hoodHubMap.put(1.8288, 2.10);

    hoodHubMap.put(1.2, 1.2);
    
    shooterHubMap.put(7.9248, 3884.0);
    
    shooterHubMap.put(7.3152, 3695.0);
    
    shooterHubMap.put(6.7056, 3505.0);

    shooterHubMap.put(6.013, 3292.0);
    
    shooterHubMap.put(5.412, 3142.0);
    
    shooterHubMap.put(4.852, 2962.0);

    shooterHubMap.put(4.25, 2872.0);
    // 12
    shooterHubMap.put(3.6576, 2862.0);
    // 11
    shooterHubMap.put(3.3528, 2762.0);
    // 10
    shooterHubMap.put(3.048, 2662.0);
    // 9
    shooterHubMap.put(2.7432, 2512.0);
    // 8
    shooterHubMap.put(2.4384, 2442.0);
    // 7
    shooterHubMap.put(2.1336, 2442.0);
    // 6
    shooterHubMap.put(1.8288, 2192.0);

    shooterHubMap.put(1.2, 2090.0);

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

    config.Slot0.kS = 0.3;
    config.Slot0.kV = 0.08;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 3.5;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicExpo_kV = 0.01;
    config.MotionMagic.MotionMagicExpo_kA = 0.01;

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

    config.Slot0.kS = 0.45;
    config.Slot0.kV = 0.09;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 2.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicExpo_kV = 0.005;
    config.MotionMagic.MotionMagicExpo_kA = 0.005;

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
    //Values for 1.5/1 gear ratio
    config.Slot0.kP = 5.742;
    config.Slot0.kS = 9.0;
    config.Slot0.kV = 0.3;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;

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

        if (Vision.isInHomeTerritory()) {

          velocity = shooterHubMap.get(currentDistance) / (60.0) + shooterSetter;

        } else {

          velocity = shooterLobMap.get(currentDistance) / (60.0) + shooterSetter;

        }
      } else {
      
        velocity = 35.6;

      }

      shooterLead.setControl(shooterRequest.withVelocity(velocity));
    });

  }

  public Command idleShooter() {
    return runOnce(() -> shooterLead.setControl(shooterRequest.withVelocity(0)));
  }

  public Command stopShooter() {
    return runOnce(() -> shooterLead.setControl(shooterRequest.withVelocity(0)));
  }

  public BooleanSupplier isAtSpeed() {
    return () -> {
      if (isTracking) {
        if(Vision.isInHomeTerritory()) {
          return shooterLead.getVelocity()
              .getValueAsDouble() > (shooterHubMap.get(Vision.getAdjustedDistance()) / (60.0) + shooterSetter) - 1;
        } else {
          return shooterLead.getVelocity()
              .getValueAsDouble() > (shooterLobMap.get(Vision.getAdjustedDistance()) / (60.0) + shooterSetter) - 1;
        }
      } else {
        return shooterLead.getVelocity().getValueAsDouble() > (35.6) - 1;
      }
    };
  }

  public double getTurretPosition() {
    return turret.getPosition().getValueAsDouble();
  }

  public boolean isTracking() {
    return isTracking;
  }
    public void logMotors() {
    double shooterLeadStatorCurrentMotor = shooterLead.getStatorCurrent().getValueAsDouble();
    double shooterLeadSupplyCurrentMotor = shooterLead.getSupplyCurrent().getValueAsDouble();
    double shooterFollowStatorCurrentMotor = shooterFollow.getStatorCurrent().getValueAsDouble();
    double shooterFollowSupplyCurrentMotor = shooterFollow.getSupplyCurrent().getValueAsDouble();
    double hoodStatorCurrentMotor = hood.getStatorCurrent().getValueAsDouble();
    double hoodSupplyCurrentMotor = hood.getSupplyCurrent().getValueAsDouble();
    double turretStatorCurrentMotor = turret.getStatorCurrent().getValueAsDouble();
    double turretSupplyCurrentMotor = turret.getSupplyCurrent().getValueAsDouble();

    SmartDashboard.putNumber("Shooter Lead Stator Motor Current", shooterLeadStatorCurrentMotor);
    SmartDashboard.putNumber("Shooter Follow Stator Motor Current", shooterFollowStatorCurrentMotor);
    SmartDashboard.putNumber("Hood Stator Motor Current", hoodStatorCurrentMotor);
    SmartDashboard.putNumber("Turret Stator Motor Current", turretStatorCurrentMotor);
    SmartDashboard.putNumber("Shooter Lead Supply Motor Current", shooterLeadSupplyCurrentMotor);
    SmartDashboard.putNumber("Shooter Follow Supply Motor Current", shooterFollowSupplyCurrentMotor);
    SmartDashboard.putNumber("Hood Supply Motor Current", hoodSupplyCurrentMotor);
    SmartDashboard.putNumber("Turret Supply Motor Current", turretSupplyCurrentMotor);

    Logger.recordOutput("Shooter Lead Stator Motor Current", shooterLeadStatorCurrentMotor);
    Logger.recordOutput("Shooter Follow Stator Motor Current", shooterFollowStatorCurrentMotor);
    Logger.recordOutput("Hood Stator Motor Current", hoodStatorCurrentMotor);
    Logger.recordOutput("Turret Stator Motor Current", turretStatorCurrentMotor);
    Logger.recordOutput("Shooter Lead Supply Motor Current", shooterLeadSupplyCurrentMotor);
    Logger.recordOutput("Shooter Follow Supply Motor Current", shooterFollowSupplyCurrentMotor);
    Logger.recordOutput("Hood Supply Motor Current", hoodSupplyCurrentMotor);
    Logger.recordOutput("Turret Supply Motor Current", turretSupplyCurrentMotor);
  }

  public void logPDH() {
    double shooterLeadCurrentPDH = pdh.getCurrent(shootLeadPDH);
    double shooterFollowCurrentPDH = pdh.getCurrent(shootFollowPDH);
    double hoodCurrentPDH = pdh.getCurrent(hoodPDH);
    double turretCurrentPDH = pdh.getCurrent(turretPDH);

    SmartDashboard.putNumber("Shooter Lead PDH Current", shooterLeadCurrentPDH);
    SmartDashboard.putNumber("Shooter Follow PDH Current", shooterFollowCurrentPDH);
    SmartDashboard.putNumber("Hood PDH Current", hoodCurrentPDH);
    SmartDashboard.putNumber("Turret PDH Current", turretCurrentPDH);

    Logger.recordOutput("Shooter Lead PDH Current", shooterLeadCurrentPDH);
    Logger.recordOutput("Shooter Follow PDH Current", shooterFollowCurrentPDH);
    Logger.recordOutput("Hood PDH Current", hoodCurrentPDH);
    Logger.recordOutput("Turret PDH Current", turretCurrentPDH);
  }

  public void logMotorTemps() {
    double shooterLeadTemp_C  = shooterLead.getDeviceTemp().getValueAsDouble();
    double shooterFollowTemp_C = shooterFollow.getDeviceTemp().getValueAsDouble();
    double hoodTemp_C         = hood.getDeviceTemp().getValueAsDouble();
    double turretTemp_C       = turret.getDeviceTemp().getValueAsDouble();

    double shooterLeadTemp_F  = (shooterLeadTemp_C  * (9.0/5.0)) + 32;
    double shooterFollowTemp_F = (shooterFollowTemp_C * (9.0/5.0)) + 32;
    double hoodTemp_F         = (hoodTemp_C         * (9.0/5.0)) + 32;
    double turretTemp_F       = (turretTemp_C       * (9.0/5.0)) + 32;

    SmartDashboard.putNumber("Shooter Lead Temp F",   shooterLeadTemp_F);
    SmartDashboard.putNumber("Shooter Follow Temp F", shooterFollowTemp_F);
    SmartDashboard.putNumber("Hood Temp F",           hoodTemp_F);
    SmartDashboard.putNumber("Turret Temp F",         turretTemp_F);

    Logger.recordOutput("Shooter Lead Temp F",   shooterLeadTemp_F);
    Logger.recordOutput("Shooter Follow Temp F", shooterFollowTemp_F);
    Logger.recordOutput("Hood Temp F",           hoodTemp_F);
    Logger.recordOutput("Turret Temp F",         turretTemp_F);
}

  @Override
  public void periodic() {
    if (isTracking && Vision.isQuestGood()) {

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
    SmartDashboard.putNumber("Shooter Hub Setpoint", shooterHubMap.get(Vision.getAdjustedDistance()));
    SmartDashboard.putNumber("Hood Hub Setpoint", hoodHubMap.get(Vision.getAdjustedDistance()));
    shooterSetter = SmartDashboard.getNumber("Shooter Added Speed", 1400);
    hoodSetter = SmartDashboard.getNumber("Hood Added Angle", 1400);

    Logger.recordOutput("Shooter/UpToSpeed", isAtSpeed());

    logMotors();
    logPDH();
    logMotorTemps();
  }

}