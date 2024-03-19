// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Climber;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  CommandXboxController m_controller;
  double leftY, rightY;
  boolean leftTrigger;
  Climber climber;

  public ClimberCommand(Climber climber, Supplier<CommandXboxController> controller) {
    addRequirements(climber);

    this.m_controller = controller.get();
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotState.isTeleop()) {
      leftY = m_controller.getLeftY();
      rightY = m_controller.getRightY();
      leftTrigger = m_controller.leftTrigger(0.5).getAsBoolean();

      if (rightY < 0) { climber.setLeftClimb((-rightY * 0.5) * (leftTrigger?1:0)); }
      else if (rightY > 0) { climber.setLeftClimb(-rightY * 0.5); }
      else { climber.setLeftClimb(0); }

      if (leftY < 0) { climber.setRightClimb((-leftY * 0.5) * (leftTrigger?1:0)); }
      else if (leftY > 0) { climber.setRightClimb(-leftY * 0.5); }
      else { climber.setRightClimb(0); }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setLeftClimb(0);
    climber.setRightClimb(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
