// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.TeamSelector;

public class AlignAndShoot extends Command {
  /** Creates a new AlignAndShoot. */
  Shooter m_shooter;
  Intake m_intake;
  AutoShoot m_autoshoot;
  CommandSwerveDrivetrain m_drivetrain;
  boolean isAligned;
  boolean reachPivot;

  public AlignAndShoot(Shooter shooter, Intake intake, AutoShoot autoshoot, CommandSwerveDrivetrain swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = shooter;
    this.m_intake = intake;
    this.m_autoshoot = autoshoot;
    this.m_drivetrain = swerve;
    isAligned = false;
    reachPivot = false;
    addRequirements(m_shooter);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAligned = false;
    reachPivot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_autoshoot.targetAuton(TeamSelector.getTeamColor() ? 4 : 7);
    if (Math.abs(m_shooter.targetAngle - m_shooter.getPivotAngle()) < 0.3) {
      m_intake.feedMotorPower(0.5);
      reachPivot = true;
    } else if (!reachPivot) {
      m_intake.feedMotorPower(0);
    }
    if (m_intake.irSensor.get() && reachPivot) {
      isAligned = true;
    }
    m_drivetrain.setControl(m_drivetrain.autoRequest
        .withSpeeds(new ChassisSpeeds(0, 0,
            m_autoshoot.rotateToTag(TeamSelector.getTeamColor() ? 4 : 7))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.feedMotorPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAligned;
  }
}
