package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  private final TalonFX spinner, kicker, diverter;
  
  private final MotionMagicVelocityVoltage spinnerRequest;
  private final MotionMagicVelocityVoltage kickerRequest;

  private final Follower diverterFollowerRequest = new Follower(Constants.Shootdexer.KICKER_CAN_ID,
      MotorAlignmentValue.Aligned);

  private States.IndexerStates indexerStates = States.IndexerStates.IDLE;

  public Indexer() {

    kicker = new TalonFX(Constants.Shootdexer.KICKER_CAN_ID);
    diverter = new TalonFX(Constants.Shootdexer.DIVERTER_CAN_ID);
    spinner = new TalonFX(Constants.Shootdexer.SPINNER_CAN_ID);

    spinnerRequest = new MotionMagicVelocityVoltage(0);
    kickerRequest = new MotionMagicVelocityVoltage(0);

    configSpinner();
    configKicker();
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

    kicker.getConfigurator().apply(config);
    diverter.getConfigurator().apply(config);
    diverter.setControl(diverterFollowerRequest);
  }

  public void setIndexerState(States.IndexerStates indexerStates) {
    this.indexerStates = indexerStates;
  }

  public void setIndexerRequests() {
    spinner.setControl(spinnerRequest.withVelocity(indexerStates.getSpinnerSpeed()));
    kicker.setControl(kickerRequest.withVelocity(indexerStates.getKickerSpeed()));
  }

  public Command setIndexer() {
    return runOnce(() -> setIndexerRequests());
  }

  @Override
  public void periodic() {
  }

}