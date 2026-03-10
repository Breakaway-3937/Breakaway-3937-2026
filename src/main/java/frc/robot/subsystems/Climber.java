package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;
import frc.robot.utility.States.ClimberStates;

public class Climber extends SubsystemBase {
  private final MotionMagicExpoVoltage climberRequest;
  private final TalonFX climber;
  private ClimberStates climberState = ClimberStates.STOW;

  public Climber() {
    climber = new TalonFX(Constants.Climber.CLIMBER_CAN_ID);

    configClimber();

    climberRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);
  }

  public void configClimber() {

    climber.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // COACH SAID COAST! DO NOT PUT IN BRAKE MODE! SOMETHING ELSE IS PASSIVELY...
    // BRAKING IT AND I'M TOO LAZY TO ASK CAD FOR A BETTER EXPLANATION!
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.2;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 8.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicExpo_kV = 0.01;
    config.MotionMagic.MotionMagicExpo_kA = 0.05;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    climber.getConfigurator().apply(config);
    climber.setPosition(0);
  }

  public void setClimberState(ClimberStates climberState) {
    this.climberState = climberState;
  }

  public Command setClimber() {
    return runOnce(() -> climber.setControl(climberRequest.withPosition(climberState.getClimb())));
  }

  @Override
  public void periodic() {

  }

}