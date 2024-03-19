// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  Shooter shooter;
  Intake intake;
  boolean rightTriggerDebounce;
  CommandSwerveDrivetrain m_drivetrain;
  CommandXboxController m_Controller;

  public ShooterCommand(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake) {
    this.m_drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    rightTriggerDebounce = false;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.targetAngle = 187;
    shooter.enableShooter = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotState.isTeleop()) {
      shooter.setPivotAngle();
      if (shooter.enableShooter) {
        shooter.setShooterVelocity();
      }else {
        shooter.stopShooter();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeMotorPower(0);
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
