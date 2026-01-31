package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.States.ClimberStates;
import frc.robot.subsystems.States.IntakeStates;

public class Intake extends SubsystemBase {
  private final TalonFX intake, intakeWrist;
  private final MotionMagicExpoVoltage intakeWristRequest;
  //private final MotionMagicVoltage intakeRequest;
  private IntakeStates intakeState = IntakeStates.STOW;

  public Intake() {
    intake = new TalonFX(Constants.Intake.INTAKE_CAN_ID);
    intakeWrist = new TalonFX(Constants.Intake.INTAKE_WRIST_CAN_ID);

    intakeWristRequest = new MotionMagicExpoVoltage(null);
    //intakeRequest = new MotionMagicVelocityVoltage();
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

  public Command turnIntakeOn() {
    return runOnce(() -> intake.set(intakeState.getPower()));
  }

  public Command turnIntakeOff() {
    return runOnce(() -> intake.set(0.0));
  }




  @Override
  public void periodic() {

  }

}