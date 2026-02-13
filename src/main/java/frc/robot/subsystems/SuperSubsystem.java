// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.States.ClimberStates;
import frc.robot.subsystems.States.IntakeStates;
import frc.robot.subsystems.States.ShootdexerStates;

public class SuperSubsystem extends SubsystemBase {

  private final Shootdexer s_Shootdexer;
  private final Intake s_Intake;
  //private final Climber s_Climber;
  

  public SuperSubsystem(Shootdexer s_Shootdexer , Intake s_Intake /*Climber s_Climber*/) {
    this.s_Shootdexer = s_Shootdexer;
    this.s_Intake = s_Intake;
    //this.s_Climber = s_Climber;
  }

  /*private Command setIntakeOut() {
    return s_Climber.setClimber().andThen(s_Intake.setIntakeWrist().alongWith(s_Intake.setIntakePower())
        .alongWith(s_Shootdexer.setKicker()).alongWith(s_Shootdexer.setSpiner()));
  }

  private Command setIntakeIn() {
    return s_Intake.setIntakeWrist().andThen(s_Shootdexer.setKicker().alongWith(s_Shootdexer.setSpiner()))
        .alongWith(s_Intake.setIntakePower()).alongWith(s_Climber.setClimber());
  }*/

  public Command autoTrack(boolean isTracking, Boolean isHub) {
    return runOnce(() -> s_Shootdexer.setAutoTracking(isTracking, isHub));
  }
/* 
  public Command fire() {
    return s_Shootdexer.runShooter();
  }
    */
  public Command spin() {
    return s_Shootdexer.runSpiner();
  }
  public Command noSpin() {
    return s_Shootdexer.stopSpiner();
  }

    public Command intake() {
    return s_Intake.runIntake();
  }
  public Command noIntake() {
    return s_Intake.stopIntake();
  }
/*
   public Command idel() {
    return s_Shootdexer.setShooterPower();
  }
*/
  /*public Command prestageClimb() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.STOW))
        .andThen(runOnce(() -> s_Shootdexer.setShootDexerState(ShootdexerStates.IDLE)))
        .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.PRESTAGE)))
        .andThen(setIntakeIn());
  }

  public Command intake() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.INTAKE))
        .andThen(runOnce(() -> s_Shootdexer.setShootDexerState(ShootdexerStates.INTAKE)))
        .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW)))
        .andThen(setIntakeOut());
  }

  public Command unclog() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.UNCLOG))
        .andThen(runOnce(() -> s_Shootdexer.setShootDexerState(ShootdexerStates.UNCLOG)))
        .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW)))
        .andThen(setIntakeOut());
  }

  public Command protectIntake() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.STOW))
        .andThen(runOnce(() -> s_Shootdexer.setShootDexerState(ShootdexerStates.IDLE)))
        .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW)))
        .andThen(setIntakeIn());
  }

  public Command overrideStow() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.STOW))
        .andThen(runOnce(() -> s_Shootdexer.setShootDexerState(ShootdexerStates.IDLE)))
        .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.STOW)))
        .andThen(setIntakeOut());
  }

  public Command climbRungOne() {
    return runOnce(() -> s_Intake.setIntakeState(IntakeStates.STOW))
        .andThen(runOnce(() -> s_Shootdexer.setShootDexerState(ShootdexerStates.IDLE)))
        .andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.RUNG_ONE))).andThen(setIntakeIn())
        .andThen(Commands.waitSeconds(0.5)).andThen(runOnce(() -> s_Climber.setClimberState(ClimberStates.PULL)))
        .andThen(setIntakeIn());
  }*/

  @Override
  public void periodic() {
  }

}
