package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX intake, intakeWrist;
  private final MotionMagicExpoVoltage intakeWristRequest;

  public Intake() {
    intake = new TalonFX(Constants.Intake.INTAKE_CAN_ID);
    intakeWrist = new TalonFX(Constants.Intake.INTAKE_WRIST_CAN_ID);

    intakeWristRequest = new MotionMagicExpoVoltage(null);
  }

  public Command setIntakeWrist() {
    return runOnce(() -> intakeWrist.setControl(intakeWristRequest.withPosition(States.IntakeStates.STOW.getAngle())));
  }

  public Command stopWrist() {
    return runOnce(() -> intakeWrist.stopMotor());
  }

  public Command turnIntakeOn() {
    return runOnce(() -> intake.set(0.1));
  }

  public Command turnIntakeOff() {
    return runOnce(() -> intake.set(0.0));
  }

    public Command unclogIntake() {
    return runOnce(() -> intake.set(-0.1));
  }


  @Override
  public void periodic() {

  }

}