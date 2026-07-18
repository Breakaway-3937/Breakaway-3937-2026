package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.Constants;
import frc.robot.utility.States;
import frc.robot.utility.States.IndexerStates;

public class Indexer extends SubsystemBase {

  private final TalonFX spinner, kicker, diverter, upsy;

  private final MotionMagicVelocityVoltage spinnerRequest;
  private final MotionMagicVelocityVoltage kickerRequest;
  private final MotionMagicVelocityVoltage upsyRequest;
  private final MotionMagicVelocityVoltage diverterRequest;

  private States.IndexerStates indexerStates = States.IndexerStates.IDLE;
  private States.IndexerStates previousState = null;
  private final int kickerPDH = 16;
  private final int diverterPDH = 14;
  private final int spinnerPDH = 4;
  private final int upsyPDH = 6;
  private final PowerDistribution pdh;
  //private final PowerDistribution pdh = new PowerDistribution(27, ModuleType.kRev);

  public Indexer(PowerDistribution pdh) {
    this.pdh = pdh;
    kicker = new TalonFX(Constants.Shootdexer.KICKER_CAN_ID);
    diverter = new TalonFX(Constants.Shootdexer.DIVERTER_CAN_ID);
    spinner = new TalonFX(Constants.Shootdexer.SPINNER_CAN_ID);
    upsy = new TalonFX(Constants.Shootdexer.UPSY_CAN_ID);

    spinnerRequest = new MotionMagicVelocityVoltage(0);
    kickerRequest = new MotionMagicVelocityVoltage(0);
    upsyRequest = new MotionMagicVelocityVoltage(0);
    diverterRequest = new MotionMagicVelocityVoltage(0);

    configSpinner();
    configKicker();
    configUpsy();
  }

  public void setState(States.IndexerStates newState) {
    System.out.println("Indexer setState called with: " + newState.toString());
    indexerStates = newState;
  }

  public States.IndexerStates getState() {
    return indexerStates;
  }

  public boolean isFiring() {
    return indexerStates == States.IndexerStates.FIRE || indexerStates == States.IndexerStates.SHUNCLOG;
  }

  private void applyFromStates(States.IndexerStates state) {
    spinner.setControl(spinnerRequest.withVelocity(state.getSpinnerSpeed()));
    kicker.setControl(kickerRequest.withVelocity(state.getKickerSpeed()));
    diverter.setControl(diverterRequest.withVelocity(state.getDiverterSpeed()));
    upsy.setControl(upsyRequest.withVelocity(state.getUpsySpeed()));
  }

  private void stopAll() {
    spinner.setControl(spinnerRequest.withVelocity(0));
    kicker.setControl(kickerRequest.withVelocity(0));
    upsy.setControl(upsyRequest.withVelocity(0));
    diverter.setControl(diverterRequest.withVelocity(0));
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

  public void configUpsy() {
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
  }

  public void logMotors() {
    double kickerStatorCurrentMotor = kicker.getStatorCurrent().getValueAsDouble();
    double diverterStatorCurrentMotor = diverter.getStatorCurrent().getValueAsDouble();
    double spinnerStatorCurrentMotor = spinner.getStatorCurrent().getValueAsDouble();
    double upsyStatorCurrentMotor = upsy.getStatorCurrent().getValueAsDouble();

    double kickerSupplyCurrentMotor = kicker.getSupplyCurrent().getValueAsDouble();
    double diverterSupplyCurrentMotor = diverter.getSupplyCurrent().getValueAsDouble();
    double spinnerSupplyCurrentMotor = spinner.getSupplyCurrent().getValueAsDouble();
    double upsySupplyCurrentMotor = upsy.getSupplyCurrent().getValueAsDouble();

    
    SmartDashboard.putNumber("Kicker Stator Motor Current", kickerStatorCurrentMotor);
    SmartDashboard.putNumber("Diverter Stator Motor Current", diverterStatorCurrentMotor);
    SmartDashboard.putNumber("Spinner Stator Motor Current", spinnerStatorCurrentMotor);
    SmartDashboard.putNumber("Upsy Stator Motor Current", upsyStatorCurrentMotor);

    SmartDashboard.putNumber("Kicker Supply Motor Current", kickerSupplyCurrentMotor);
    SmartDashboard.putNumber("Diverter Supply Motor Current", diverterSupplyCurrentMotor);
    SmartDashboard.putNumber("Spinner Supply Motor Current", spinnerSupplyCurrentMotor);
    SmartDashboard.putNumber("Upsy Supply Motor Current", upsySupplyCurrentMotor);

    Logger.recordOutput("Kicker Stator Motor Current", kickerStatorCurrentMotor);
    Logger.recordOutput("Diverter Stator Motor Current", diverterStatorCurrentMotor);
    Logger.recordOutput("Spinner Stator Motor Current", spinnerStatorCurrentMotor);
    Logger.recordOutput("Upsy Stator Motor Current", upsyStatorCurrentMotor);

    Logger.recordOutput("Kicker Supply Motor Current", kickerSupplyCurrentMotor);
    Logger.recordOutput("Diverter Supply Motor Current", diverterSupplyCurrentMotor);
    Logger.recordOutput("Spinner Supply Motor Current", spinnerSupplyCurrentMotor);
    Logger.recordOutput("Upsy Supply Motor Current", upsySupplyCurrentMotor);

  }

  public void logPDH() {
    double kickerCurrentPDH = pdh.getCurrent(kickerPDH);
    double diverterCurrentPDH = pdh.getCurrent(diverterPDH);
    double spinnerCurrentPDH = pdh.getCurrent(spinnerPDH);
    double upsyCurrentPDH = pdh.getCurrent(upsyPDH);

    SmartDashboard.putNumber("Kicker PDH Current", kickerCurrentPDH);
    SmartDashboard.putNumber("Diverter PDH Current", diverterCurrentPDH);
    SmartDashboard.putNumber("Spinner PDH Current", spinnerCurrentPDH);
    SmartDashboard.putNumber("Upsy PDH Current", upsyCurrentPDH);

    Logger.recordOutput("Kicker PDH Current", kickerCurrentPDH);
    Logger.recordOutput("Diverter PDH Current", diverterCurrentPDH);
    Logger.recordOutput("Spinner PDH Current", spinnerCurrentPDH);
    Logger.recordOutput("Upsy PDH Current", upsyCurrentPDH);
  }

  public void logMotorTemps() {
    double kickerTemp_C = kicker.getDeviceTemp().getValueAsDouble();
    double diverterTemp_C = diverter.getDeviceTemp().getValueAsDouble();
    double spinnerTemp_C = spinner.getDeviceTemp().getValueAsDouble();
    double upsyTemp_C = upsy.getDeviceTemp().getValueAsDouble();

    double kickerTemp_F = ((kickerTemp_C * (9.0/5.0)) +32);
    double diverterTemp_F = ((diverterTemp_C * (9.0/5.0)) +32);
    double spinnerTemp_F = ((spinnerTemp_C * (9.0/5.0)) +32);
    double upsyTemp_F = ((upsyTemp_C * (9.0/5.0)) +32);

    SmartDashboard.putNumber("Kicker Temp F", kickerTemp_F);
    SmartDashboard.putNumber("Diverter Temp F", diverterTemp_F);
    SmartDashboard.putNumber("Spinner Temp F", spinnerTemp_F);
    SmartDashboard.putNumber("Upsy Temp F", upsyTemp_F);

    Logger.recordOutput("Kicker Temp F", kickerTemp_F);
    Logger.recordOutput("Diverter Temp F", diverterTemp_F);
    Logger.recordOutput("Spinner Temp F", spinnerTemp_F);
    Logger.recordOutput("Upsy Temp F", upsyTemp_F);


  }
  /*
   * 
   * public void setIndexerState(States.IndexerStates indexerStates) {
   * this.indexerStates = indexerStates;
   * }
   */
  /*
   * public void setIndexerRequests() {
   * spinner.setControl(spinnerRequest.withVelocity(indexerStates.getSpinnerSpeed(
   * )));
   * kicker.setControl(kickerRequest.withVelocity(indexerStates.getKickerSpeed()))
   * ;
   * diverter.setControl(diverterRequest.withVelocity(indexerStates.getKickerSpeed
   * ()));
   * upsy.setControl(upsyRequest.withVelocity(indexerStates.getUpsySpeed()));
   * }
   * 
   * public void setIndexerRequestsStop() {
   * spinner.setControl(spinnerRequest.withVelocity(0));
   * kicker.setControl(kickerRequest.withVelocity(0));
   * upsy.setControl(kickerRequest.withVelocity(0));
   * diverter.setControl(diverterRequest.withVelocity(0));
   * }
   * 
   * public Command setIndexer() {
   * return runOnce(() -> setIndexerRequests());
   * }
   * 
   * public Command stopIndexer() {
   * return runOnce(() -> setIndexerRequestsStop());
   * }
   * 
   * private void reverseDiverter() {
   * diverter.setControl(diverterRequest.withVelocity(-10));
   * }
   * 
   * private WaitUntilCommand waitForTime(double time) {
   * return new WaitUntilCommand(time);
   * }
   * 
   * private Command fixDiverterCommand() {
   * return runOnce(() -> reverseDiverter())
   * .andThen(waitForTime(0.25))
   * .andThen(setIndexer());
   * }
   */

  @Override
  public void periodic() {
    boolean stateChanged = (indexerStates != previousState);

    switch (indexerStates) {

      case IDLE:
        if (stateChanged) {
          stopAll();
        }
        break;

      case FIRE:

        applyFromStates(IndexerStates.FIRE);

        break;

      case SHUNCLOG:

        applyFromStates(IndexerStates.SHUNCLOG);

        break;
    }

    previousState = indexerStates;

    SmartDashboard.putString("Indexer State", indexerStates.toString());
    Logger.recordOutput("Indexer State", indexerStates.toString());
    logMotors();
    logPDH();
    logMotorTemps();

  }

}