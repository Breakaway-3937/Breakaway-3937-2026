package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.Constants;
import frc.robot.utility.States;

public class Indexer extends SubsystemBase {

  private final TalonFX spinner, kicker, diverter/* , upsy*/;

  private final MotionMagicVelocityVoltage spinnerRequest;
  private final MotionMagicVelocityVoltage kickerRequest;
  //private final MotionMagicVelocityVoltage upsyRequest;
  private final MotionMagicVelocityVoltage diverterRequest;

  private States.IndexerStates indexerStates = States.IndexerStates.IDLE;

  public Indexer() {

    kicker = new TalonFX(Constants.Shootdexer.KICKER_CAN_ID);
    diverter = new TalonFX(Constants.Shootdexer.DIVERTER_CAN_ID);
    spinner = new TalonFX(Constants.Shootdexer.SPINNER_CAN_ID);
    //upsy = new TalonFX(Constants.Shootdexer.UPSY_CAN_ID);

    spinnerRequest = new MotionMagicVelocityVoltage(0);
    kickerRequest = new MotionMagicVelocityVoltage(0);
    //upsyRequest = new MotionMagicVelocityVoltage(0);
    diverterRequest = new MotionMagicVelocityVoltage(0);

    configSpinner();
    configKicker();
    //configUpsy();
  }

  public void configSpinner() {

    spinner.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.07;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 900;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20;

    spinner.getConfigurator().apply(config);
  }

  public void configKicker() {
    kicker.getConfigurator().apply(new TalonFXConfiguration());
    diverter.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.07;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 900;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;

    kicker.getConfigurator().apply(config);
    diverter.getConfigurator().apply(config);
  }

 /*  public void configUpsy() {
    upsy.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.07;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 900;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20;

    upsy.getConfigurator().apply(config);
  }*/

  public void setIndexerState(States.IndexerStates indexerStates) {
    this.indexerStates = indexerStates;
  }

  public void setIndexerRequests() {
    spinner.setControl(spinnerRequest.withVelocity(indexerStates.getSpinnerSpeed()));
    kicker.setControl(kickerRequest.withVelocity(indexerStates.getKickerSpeed()));
    diverter.setControl(diverterRequest.withVelocity(indexerStates.getKickerSpeed()));
    //upsy.setControl(upsyRequest.withVelocity(indexerStates.getUpsySpeed()));
  }

  public void setIndexerRequestsStop() {
    spinner.setControl(spinnerRequest.withVelocity(0));
    kicker.setControl(kickerRequest.withVelocity(0));
    //upsy.setControl(kickerRequest.withVelocity(0));
    diverter.setControl(diverterRequest.withVelocity(0));
  }

  public Command setIndexer() {
    return runOnce(() -> setIndexerRequests());
  }

  public Command stopIndexer() {
    return runOnce(() -> setIndexerRequestsStop());
  }

  private void reverseDiverter() {
    diverter.setControl(diverterRequest.withVelocity(-10));
  }

  private WaitUntilCommand waitForTime(double time) {
    return new WaitUntilCommand(time);
  }

  private Command fixDiverterCommand() {
    return runOnce(() -> reverseDiverter())
      .andThen(waitForTime(0.25))
      .andThen(setIndexer());
  }

  @Override
  public void periodic() {
   /*  SmartDashboard.putNumber("Diverter MotorVoltage", diverter.getSupplyCurrent().getValueAsDouble());
    if(diverter.getSupplyCurrent().getValueAsDouble() > 20.0) {
      fixDiverterCommand();
    }*/
  }

}