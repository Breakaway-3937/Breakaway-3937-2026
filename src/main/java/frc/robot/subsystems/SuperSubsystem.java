// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.States.ClimberStates;
import frc.robot.utility.States.IndexerStates;
import frc.robot.utility.States.IntakeStates;

public class SuperSubsystem extends SubsystemBase {

  private final Shooter s_Shooter;
  private final Indexer s_Indexer;
  private final Intake s_Intake;
  // private final Climber s_Climber;
  ParallelCommandGroup runSubsystems;
  ParallelCommandGroup runSubsystems2;
  ParallelCommandGroup idleSubsystems;

  public SuperSubsystem(Shooter s_Shooter, Indexer s_Indexer, Intake s_Intake /* Climber s_Climber */) {
    this.s_Shooter = s_Shooter;
    this.s_Indexer = s_Indexer;
    this.s_Intake = s_Intake;
    // this.s_Climber = s_Climber;

    runSubsystems = new ParallelCommandGroup(s_Indexer.setIndexer(), s_Intake.setIntake());
    runSubsystems2 = new ParallelCommandGroup(s_Indexer.setIndexer(), s_Intake.setIntake());
    idleSubsystems = new ParallelCommandGroup(s_Shooter.idleShooter(), s_Indexer.setIndexer(), s_Intake.setIntake());
  }

  private Command setIntakeOut() {
    return /* s_Climber.setClimber().andThen( */s_Intake.setIntake();
  }
  
  private Command setIntakeIn() {
    return s_Intake.setIntake()/* .andThen(s_Climber.setClimber()) */;
  }

  public Command autoTrack(boolean isTracking, Boolean isHub) {
    return runOnce(() -> s_Shooter.setAutoTracking(isTracking, isHub));
  }

  public Command fire() {
    return runOnce(() -> s_Indexer.setIndexerState(IndexerStates.FIRE))
      .andThen(runOnce(() -> s_Intake.setIntakeState(IntakeStates.FIRE)))
      .andThen(s_Shooter.runShooter())
      .andThen(runSubsystems);
  }

  public Command idle() {
    return runOnce(() -> s_Indexer.setIndexerState(IndexerStates.IDLE))
      .andThen(runOnce(() -> s_Intake.setIntakeState(IntakeStates.IDLE)))
      .andThen(idleSubsystems);
  }

  public Command combo() {
    return runOnce(() -> s_Indexer.setIndexerState(IndexerStates.FIRE))
      .andThen(runOnce(() -> s_Intake.setIntakeState(IntakeStates.INTAKE)))
      .andThen(s_Shooter.runShooter())
      .andThen(runSubsystems2);
  }

  /*
   * public Command prestageClimb() {
   * return runOnce(() -> s_Intake.setIntakeState(IntakeStates.STOW))
   * .andThen(runOnce(() ->
   * s_Shootdexer.setShootDexerState(ShootdexerStates.IDLE)))
   * .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.PRESTAGE)))
   * .andThen(setIntakeIn());
   * }
   */

  public Command intake() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.INTAKE))
        /* .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW))) */
        .andThen(setIntakeOut());
  }

  public Command stopIntake() {
    return s_Intake.stopIntake();
  }

  public Command unclog() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.UNCLOG))
        /* .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW))) */
        .andThen(setIntakeOut());
  }

  public Command protectIntake() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.STOW))
        .andThen(runOnce(() -> s_Indexer.setIndexerState(IndexerStates.IDLE)))
        /* .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW))) */
        .andThen(s_Intake.setIntake())
        .alongWith(s_Indexer.setIndexer());
  }

  public Command overrideStow() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.STOW))
        .andThen(runOnce(() -> s_Indexer.setIndexerState(IndexerStates.IDLE)))
        /* .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW))) */
        .andThen(setIntakeOut());
  }
  /*
   * public Command climbRungOne() {
   * return runOnce(() -> s_Intake.setIntakeState(IntakeStates.STOW))
   * .andThen(runOnce(() ->
   * s_Shootdexer.setShootDexerState(ShootdexerStates.IDLE)))
   * .andThen(runOnce(() ->
   * s_Climber.setClimberState(ClimberStates.RUNG_ONE))).andThen(setIntakeIn())
   * .andThen(Commands.waitSeconds(0.5)).andThen(runOnce(() ->
   * s_Climber.setClimberState(ClimberStates.PULL)))
   * .andThen(setIntakeIn());
   * }
   */

  @Override
  public void periodic() {
    //System.out.println("Un-comment this to immediately spike the ram usage.");
    CommandScheduler.getInstance().schedule(runSubsystems.onlyIf(s_Shooter.isAtSpeed()));
  }

}
