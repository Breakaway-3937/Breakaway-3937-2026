// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shootdexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShootdexer extends Command {

  private final Shootdexer s_Shootdexer;
  private final CommandXboxController xboxController;

  public RunShootdexer(Shootdexer s_Shootdexer, CommandXboxController xboxController) {
    this.s_Shootdexer = s_Shootdexer;
    this.xboxController = xboxController;
    addRequirements(s_Shootdexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shootdexer.setAutoTracking(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.start().getAsBoolean()) {
      s_Shootdexer.setAutoTracking(true);
    }
    if(xboxController.back().getAsBoolean()) {
      s_Shootdexer.setAutoTracking(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
