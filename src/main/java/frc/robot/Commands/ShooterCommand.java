// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.generated.TunerConstants;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  Shooter shooter;
  Intake intake;
  Supplier<Boolean> bButtonSupplier;
  Supplier<Boolean> yButtonSupplier;
  Supplier<Boolean> aButtonSupplier;
  boolean rightTriggerDebounce;
  private final XboxController drivercontroller = new XboxController(0);
  CommandSwerveDrivetrain m_drivetrain;

  public ShooterCommand(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake, Supplier<Boolean> bButtonSupplier, Supplier<Boolean> yButtonSupplier, Supplier<Boolean> aButtonSupplier) {
    this.m_drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.bButtonSupplier = bButtonSupplier;
    this.yButtonSupplier = yButtonSupplier;
    this.yButtonSupplier = aButtonSupplier;
    rightTriggerDebounce = false;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.targetAngle = 187;
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotState.isTeleop()) {
      if (drivercontroller.getRightTriggerAxis() > 0.5) {
        if (!rightTriggerDebounce) {
          rightTriggerDebounce = true;
          m_drivetrain.seedFieldRelative(new Pose2d());
          
        }
      } else {
        rightTriggerDebounce = false;
      }
      shooter.setPivotAngle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeMotorPower(0);
    shooter.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
