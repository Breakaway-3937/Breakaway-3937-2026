// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.utility.States.ClimberStates;
import frc.robot.utility.States.IndexerStates;
import frc.robot.utility.States.IntakeStates;

public class SuperSubsystem extends SubsystemBase {

  public enum RobotState {
    IDLE,
    IDLE_INTAKE_DOWN,
    INTAKING,
    UNCLOGGING,
    SPINNING_UP,
    FIRING,
    COMBO_FIRING,
    SHUNCLOG_FIRING,
    SHUNCLOG,
    COMBO,
    AUTO_COMBO,
    PROTECT_INTAKE,
    OVERRIDE_STOW
  }

  private final Shooter s_Shooter;
  private final Indexer s_Indexer;
  private final Intake s_Intake;
  private final Vision s_Vision;

  PowerDistribution pdp = new PowerDistribution(27, ModuleType.kRev);

  private RobotState currentState = RobotState.IDLE;
  private RobotState previousState = null;
  private Command shooterCommand = null;


  public SuperSubsystem(Shooter s_Shooter, Indexer s_Indexer, Intake s_Intake, Vision s_Vision) {
    this.s_Shooter = s_Shooter;
    this.s_Indexer = s_Indexer;
    this.s_Intake = s_Intake;
    this.s_Vision = s_Vision;
  }

  public void setState(RobotState newState) {
    currentState = newState;
  }

  public RobotState getState() {
    return currentState;
  }

  private void scheduleShooter() {
    cancelShooter();
    shooterCommand = s_Shooter.runShooter().repeatedly();
    CommandScheduler.getInstance().schedule(shooterCommand);
  }

  private void cancelShooter() {
    if (shooterCommand != null) {
      shooterCommand.cancel();
      shooterCommand = null;
    }
  }

  private void transitionTo(RobotState newState) {
    currentState = newState;
    previousState = null;
  }

  public Command autoTrack(boolean isTracking) {
    return runOnce(() -> s_Shooter.setAutoTracking(isTracking));
  }

  public Command fire() {
    return runOnce(() -> setState(RobotState.SPINNING_UP));
  }

  public Command shunClog() {
    return runOnce(() -> setState(RobotState.SHUNCLOG));
  }

  private boolean isShootingState(RobotState state) {
    return state == RobotState.SPINNING_UP
        || state == RobotState.FIRING
        || state == RobotState.COMBO_FIRING
        || state == RobotState.SHUNCLOG_FIRING
        || state == RobotState.SHUNCLOG
        || state == RobotState.COMBO
        || state == RobotState.AUTO_COMBO;
}

public Command idle() {
    return runOnce(() -> {
        if (!isShootingState(currentState)) {
            setState(RobotState.IDLE);
        }
    });
}

public Command idleWithIntakeDown() {
    return runOnce(() -> {
        if (!isShootingState(currentState)) {
            setState(RobotState.IDLE_INTAKE_DOWN);
        }
    });
}

  public Command intake() {
    return runOnce(() -> setState(RobotState.INTAKING));
  }

  public Command unclog() {
    return runOnce(() -> setState(RobotState.UNCLOGGING));
  }

  public Command combo() {
    return runOnce(() -> setState(RobotState.COMBO));
  }

  public Command autoCombo() {
    return runOnce(() -> setState(RobotState.AUTO_COMBO));
  }

  public Command protectIntake() {
    return runOnce(() -> setState(RobotState.PROTECT_INTAKE));
  }

  public Command overrideStow() {
    return runOnce(() -> setState(RobotState.OVERRIDE_STOW));
  }

  @Override
  public void periodic() {
    boolean stateChanged = (currentState != previousState);

    


      RobotContainer.setMultipliers(isShootingState(currentState) ? 0.3 : 1.0);
    

    switch (currentState) {

      case IDLE:
        if (stateChanged) {
          cancelShooter();
          s_Indexer.setState(IndexerStates.IDLE);
          s_Intake.setState(IntakeStates.IDLE);
          CommandScheduler.getInstance().schedule(s_Shooter.idleShooter());
        }
        break;

      case IDLE_INTAKE_DOWN:
        if (stateChanged) {
          cancelShooter();
          s_Indexer.setState(IndexerStates.IDLE);
          s_Intake.setState(IntakeStates.INTAKE_IDLE);
          CommandScheduler.getInstance().schedule(s_Shooter.idleShooter());
        }
        break;

      case INTAKING:
        if (stateChanged) {
          cancelShooter();
          s_Intake.setState(IntakeStates.INTAKE);
        }
        break;

      case UNCLOGGING:
        if (stateChanged) {
          cancelShooter();
          s_Intake.setState(IntakeStates.UNCLOG);
        }
        break;

      case SPINNING_UP:
        if (stateChanged) {
          s_Indexer.setState(IndexerStates.IDLE);
          scheduleShooter();
        }
        if (s_Shooter.isAtSpeed().getAsBoolean() && s_Vision.isTurretSafe().getAsBoolean()) {
          transitionTo(RobotState.FIRING);
        }
        break;

      case FIRING:

        s_Intake.setState(IntakeStates.FIRE);
        s_Indexer.setState(IndexerStates.FIRE);

        break;
      case COMBO_FIRING:

        s_Intake.setState(IntakeStates.INTAKE);
        s_Indexer.setState(IndexerStates.FIRE);

        break;
      case SHUNCLOG_FIRING:

        s_Intake.setState(IntakeStates.UNCLOG);
        s_Indexer.setState(IndexerStates.SHUNCLOG);

        break;

      case SHUNCLOG:
        if (stateChanged) {
          s_Indexer.setState(IndexerStates.IDLE);
          s_Intake.setState(IntakeStates.UNCLOG);
          scheduleShooter();
        }
        if (s_Shooter.isAtSpeed().getAsBoolean() && s_Vision.isTurretSafe().getAsBoolean()) {
          transitionTo(RobotState.FIRING);
        }
        break;

      case COMBO:

        s_Indexer.setState(IndexerStates.IDLE);
        s_Intake.setState(IntakeStates.INTAKE);
        scheduleShooter();

        if (s_Shooter.isAtSpeed().getAsBoolean() && s_Vision.isTurretSafe().getAsBoolean()) {
          transitionTo(RobotState.COMBO_FIRING);
        }
        break;

      case AUTO_COMBO:

        s_Indexer.setState(IndexerStates.IDLE);
        s_Intake.setState(IntakeStates.INTAKE);
        scheduleShooter();

        if (s_Vision.isTurretSafe().getAsBoolean()) {
          transitionTo(RobotState.COMBO_FIRING);
        }
        break;

      case PROTECT_INTAKE:
        if (stateChanged) {
          cancelShooter();
          s_Intake.setState(IntakeStates.STOW);
          s_Indexer.setState(IndexerStates.IDLE);
        }
        break;

      case OVERRIDE_STOW:
        if (stateChanged) {
          cancelShooter();
          s_Intake.setState(IntakeStates.STOW);
          s_Indexer.setState(IndexerStates.IDLE);
        }
        break;
    }

    previousState = currentState;

    /*
     * SmartDashboard.putNumber("Turret Amps", pdp.getCurrent(12));
     * SmartDashboard.putNumber("Kicker Amps", pdp.getCurrent(13));
     * SmartDashboard.putNumber("Diverter Amps", pdp.getCurrent(14));
     * SmartDashboard.putNumber("Shooter Lead Amps", pdp.getCurrent(15));
     * SmartDashboard.putNumber("Spinner Amps", pdp.getCurrent(4));
     */
    SmartDashboard.putBoolean("isAtSpeed", s_Shooter.isAtSpeed().getAsBoolean());
    SmartDashboard.putBoolean("isTurretSafe", s_Vision.isTurretSafe().getAsBoolean());
    SmartDashboard.putString("Indexer State Raw", s_Indexer.getState().toString());
    SmartDashboard.putString("Intake State Raw", s_Intake.getState().toString());
    SmartDashboard.putString("Robot State", currentState.toString());

  }
}