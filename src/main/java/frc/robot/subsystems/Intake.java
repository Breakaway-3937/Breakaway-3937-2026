package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;
import frc.robot.utility.States.IntakeStates;

public class Intake extends SubsystemBase {
  private final TalonFX intakeWrist;
  private final TalonFX intake;
  private final MotionMagicExpoVoltage intakeWristRequest;
  private final MotionMagicVelocityVoltage intakeRequest;
  private IntakeStates intakeState = IntakeStates.STOW;

  public Intake() {
    intake = new TalonFX(Constants.Intake.INTAKE_CAN_ID);
    intakeWrist = new TalonFX(Constants.Intake.INTAKE_WRIST_CAN_ID);

    configIntakeWrist();
    configIntake();

    intakeWristRequest = new MotionMagicExpoVoltage(0);
    intakeRequest = new MotionMagicVelocityVoltage(0);

    //super.setDefaultCommand(setIntake());
  }

  public void configIntakeWrist() {
    intakeWrist.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.15;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 2.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    //FIXME

    config.MotionMagic.MotionMagicExpo_kV = 0.7;
    config.MotionMagic.MotionMagicExpo_kA = 0.01;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;

    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    intakeWrist.getConfigurator().apply(config);
    intakeWrist.setPosition(0);
  }

  public void configIntake() {
    intake.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.00;
    config.Slot0.kP = 0.07;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 400;
    config.MotionMagic.MotionMagicJerk = 4000;

    intake.getConfigurator().apply(config);
  }

  public void setIntakeState(IntakeStates intakeState) {
    this.intakeState = intakeState;
  }

  public Command setIntakeWrist() {
    return runOnce(() -> intakeWrist.setControl(intakeWristRequest.withPosition(intakeState.getAngle())));
  }

  public Command setIntakePower() {
    return runOnce(() -> intake.setControl(intakeRequest.withVelocity(intakeState.getPower())));
  }

  public void setIntakeRequests() {
    intakeWrist.setControl(intakeWristRequest.withPosition(intakeState.getAngle()));
    intake.setControl(intakeRequest.withVelocity(intakeState.getPower()));
  }

  public Command setIntake() {
    return runOnce(() -> setIntakeRequests());
  }

  public Command stopWrist() {
    return runOnce(() -> intakeWrist.stopMotor());
  }

  public Command stopIntake() {
    return runOnce(() -> intake.stopMotor());
  }

  @Override
  public void periodic() {
  }

}