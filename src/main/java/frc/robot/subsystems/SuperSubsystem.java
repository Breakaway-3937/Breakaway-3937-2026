// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.utility.States.ClimberStates;
import frc.robot.utility.States.IndexerStates;
import frc.robot.utility.States.IntakeStates;

public class SuperSubsystem extends SubsystemBase {

  private final Shooter s_Shooter;
  private final Indexer s_Indexer;
  private final Intake s_Intake;
  private final Climber s_Climber;
  private final Vision s_Vision;

  //BooleanSupplier shooterGood;
  BooleanSupplier bool = () -> true;

  PowerDistribution pdp = new PowerDistribution(27, ModuleType.kRev);

  public SuperSubsystem(Shooter s_Shooter, Indexer s_Indexer, Intake s_Intake, Climber s_Climber, Vision s_Vision) {
    this.s_Shooter = s_Shooter;
    this.s_Indexer = s_Indexer;
    this.s_Intake = s_Intake;
    this.s_Climber = s_Climber;
    this.s_Vision = s_Vision;

    //shooterGood = () -> s_Vision.isTurretSafe();
  }

  private Command setIntakeOut() {
    return /* s_Climber.setClimber().andThen( */s_Intake.setIntake();
  }

  private Command setIntakeIn() {
    return s_Intake.setIntake()/* .andThen(s_Climber.setClimber()) */;
  }

  private ParallelCommandGroup runSubsystems() {
    return new ParallelCommandGroup(s_Indexer.setIndexer(), s_Intake.setIntake());
  }

  private ParallelCommandGroup idleSubsystems() {
    return new ParallelCommandGroup(s_Shooter.idleShooter(), s_Indexer.setIndexer(), s_Intake.setIntake());
  }

  private ParallelCommandGroup idleSubsystemsWithIntakeDown() {
    return new ParallelCommandGroup(s_Shooter.idleShooter(), s_Indexer.setIndexer(), s_Intake.stopIntake());
  }

  private WaitUntilCommand waitForShooterSpeed() {
    return new WaitUntilCommand(s_Shooter.isAtSpeed());
  }

  private ConditionalCommand runSubsystemsIfSafe() {
    return new ConditionalCommand(runSubsystems(), stopUnsafe(), s_Vision.isTurretSafe());
  }

  private Command stopUnsafe() {
    return runOnce(() -> s_Indexer.stopIndexer());
  }

  public Command autoTrack(boolean isTracking) {
    return runOnce(() -> s_Shooter.setAutoTracking(isTracking));
  }

  public Command fire() {
    return runOnce(() -> s_Indexer.setIndexerState(IndexerStates.FIRE))
        .andThen(runOnce(() -> s_Intake.setIntakeState(IntakeStates.FIRE)))
        .andThen(s_Shooter.runShooter()
        .andThen(RobotContainer.setMultipliers(0.4))
        .andThen(waitForShooterSpeed())
        .andThen(runSubsystemsIfSafe()));
  }

  public Command idle() {
    return runOnce(() -> s_Indexer.setIndexerState(IndexerStates.IDLE))
        .andThen(runOnce(() -> s_Intake.setIntakeState(IntakeStates.IDLE)))
        .andThen(RobotContainer.setMultipliers(1.0))
        .andThen(idleSubsystems());
  }

  public Command combo() {
    return runOnce(() -> s_Indexer.setIndexerState(IndexerStates.FIRE))
        .andThen(runOnce(() -> s_Intake.setIntakeState(IntakeStates.INTAKE)))
        .andThen(s_Shooter.runShooter())
        .andThen(RobotContainer.setMultipliers(0.4))
        .andThen(waitForShooterSpeed())
        .andThen(runSubsystemsIfSafe());
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

  public Command pullClimber() {
    return runOnce(() -> s_Climber.setClimberState(ClimberStates.PULL))
        .andThen(s_Climber.setClimber());
  }

  public Command raiseClimber() {
    return runOnce(() -> s_Climber.setClimberState(ClimberStates.RUNG_ONE))
        .andThen(s_Climber.setClimber());
  }

  public Command stowClimber() {
    return runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW))
        .andThen(s_Climber.setClimber());
  }

  public Command intake() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.INTAKE))
        /* .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW))) */
        .andThen(setIntakeOut());
  }

  public Command idleWithIntakeDown() {
    return runOnce(() -> s_Indexer.setIndexerState(IndexerStates.IDLE))
        .andThen(RobotContainer.setMultipliers(0.4))
        .andThen(idleSubsystemsWithIntakeDown());
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
    // System.out.println("Un-comment this to immediately spike the ram usage.");

    /*SmartDashboard.putNumber("Turret Amps", pdp.getCurrent(12));
    SmartDashboard.putNumber("Kicker Amps", pdp.getCurrent(13));
    SmartDashboard.putNumber("Diverter Amps", pdp.getCurrent(14));
    SmartDashboard.putNumber("Shooter Lead Amps", pdp.getCurrent(15));
    SmartDashboard.putNumber("Spinner Amps", pdp.getCurrent(4));*/
    
  }

}
