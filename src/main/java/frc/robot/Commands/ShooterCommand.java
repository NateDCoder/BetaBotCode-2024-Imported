// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  Shooter shooter;
  Intake intake;
  Supplier<Double> leftYSupplier;
  Supplier<Boolean> bButtonSupplier;
  Supplier<Boolean> yButtonSupplier;

  public ShooterCommand(Shooter shooter, Intake intake, Supplier<Double> leftYSupplier, Supplier<Boolean> bButtonSupplier,
      Supplier<Boolean> yButtonSupplier) {
    this.shooter = shooter;
    this.intake = intake;
    this.leftYSupplier = leftYSupplier;
    this.bButtonSupplier = bButtonSupplier;
    this.yButtonSupplier = yButtonSupplier;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.pivotRawPower(-leftYSupplier.get());
    if (RobotState.isTeleop()) {
      shooter.setPivotAngle();

      // Set intake power to - on y button, and + on b button
      intake.intakeMotorPower(
        (yButtonSupplier.get()?-Constants.INTAKE_POWER:0) +
        (bButtonSupplier.get()?Constants.INTAKE_POWER:0));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
