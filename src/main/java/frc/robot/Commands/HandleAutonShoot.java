// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class HandleAutonShoot extends Command {
  /** Creates a new HandleAutonShoot. */
  Intake intake;
  Shooter shooter;
  boolean isNote;
  public HandleAutonShoot(Intake intake, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.shooter = shooter;
    isNote = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.irSensor.get()) {
      intake.feedMotorPower(Constants.FEED_POWER);
      intake.intakeMotorPower(Constants.INTAKE_POWER);
    }else {
      isNote = true;
      intake.feedMotorPower(0.0);
      intake.intakeMotorPower(0.0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.feedMotorPower(0);
    intake.intakeMotorPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isNote;
  }
}
