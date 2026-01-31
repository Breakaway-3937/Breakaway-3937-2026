// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperSubsystem extends SubsystemBase {

  private final Shootdexer s_Shootdexer;
  private final Intake s_Intake;
  private final Climber s_Climber;
  private final Vision s_Vision;
  private boolean isAutoTracking = true;

  public SuperSubsystem(Shootdexer s_Shootdexer, Intake s_Intake, Climber s_Climber, Vision s_Vision) {
    this.s_Shootdexer = s_Shootdexer;
    this.s_Intake = s_Intake;
    this.s_Climber = s_Climber;
    this.s_Vision = s_Vision;
  }

  public Command autoTrack(boolean isTracking, Boolean isHub) {
    return runOnce(() -> s_Shootdexer.setAutoTracking(isTracking, isHub));
  }

  public Command fire() {
    return runOnce(() -> s_Shootdexer.runShooter());
  }

  @Override
  public void periodic() {}

}
