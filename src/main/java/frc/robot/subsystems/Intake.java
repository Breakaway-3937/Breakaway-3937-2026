package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.States.IntakeStates;

public class Intake extends SubsystemBase {
  private final TalonFX intake, intakeWrist;
  private final MotionMagicExpoVoltage intakeWristRequest;
  private IntakeStates intakeState = IntakeStates.STOW;

  public Intake() {
    intake = new TalonFX(Constants.Intake.INTAKE_CAN_ID);
    intakeWrist = new TalonFX(Constants.Intake.INTAKE_WRIST_CAN_ID);

    configIntakeWrist();
    configIntake();

    intakeWristRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);
  }

  public void configIntakeWrist() {
    intakeWrist.getConfigurator().apply(new TalonFXConfiguration());

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

    intakeWrist.getConfigurator().apply(config);
    intakeWrist.setPosition(0);
  }

  public void configIntake() {
    intake.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    intake.getConfigurator().apply(config);
  }

  public void setIntakeState(IntakeStates intakeState) {
    this.intakeState = intakeState;
  }

  public Command setIntakeWrist() {
    return runOnce(() -> intakeWrist.setControl(intakeWristRequest.withPosition(intakeState.getAngle())));
  }

  public Command stopWrist() {
    return runOnce(() -> intakeWrist.stopMotor());
  }

  public Command setIntakePower() {
    return runOnce(() -> intake.set(intakeState.getPower()));
  }

  @Override
  public void periodic() {

  }

}