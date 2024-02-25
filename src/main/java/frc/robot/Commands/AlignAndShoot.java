// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class AlignAndShoot extends Command {
  /** Creates a new AlignAndShoot. */
  Shooter m_shooter;
  Intake m_intake;
  AutoShoot m_autoshoot;
  boolean isAligned;
  public AlignAndShoot(Shooter shooter, Intake intake, AutoShoot autoshoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = shooter;
    this.m_intake = intake;
    this.m_autoshoot = autoshoot;
    isAligned = false;
    addRequirements(m_shooter);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_autoshoot.targetAll(7);
    if(Math.abs(m_shooter.targetAngle-m_shooter.getPivotAngle()) < 1) {
      m_intake.feedMotorPower(0.6);
      isAligned = true;
    }else {
      m_intake.feedMotorPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAligned;
  }
}
