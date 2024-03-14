// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  Supplier<Double> leftYSupplier, rightYSupplier;
  Supplier<Boolean> leftTriggerSupplier;
  Climber climber;

  public ClimberCommand(Climber climber, Supplier<Double> leftYSupplier, Supplier<Double> rightYSupplier, Supplier<Boolean> leftTriggerSupplier) {
    addRequirements(climber);
    this.leftYSupplier = leftYSupplier;
    this.rightYSupplier = rightYSupplier;
    this.climber = climber;
    this.leftTriggerSupplier = leftTriggerSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotState.isTeleop()) {
      // if lefty is above or below deadband and the driver is pressing b run the climbers based on operator input
      if (Math.abs(leftYSupplier.get()) > 0.1 && leftTriggerSupplier.get()) {
        climber.setLeftClimb(rightYSupplier.get() * 0.2);
        climber.setRightClimb(leftYSupplier.get() * 0.2);
      }

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
