// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class EitherCommand extends Command {
  /** Creates a new EitherCommand. */
  Command command1, command2;
  Supplier<Boolean> bool;
  public EitherCommand(Command comand1, Command comand2, Supplier<Boolean> bool) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.command1 = comand1;
    this.command2 = comand2;
    this.bool = bool;
  }
  public Command getCommand() {
    return bool.get()? command1: command2;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
