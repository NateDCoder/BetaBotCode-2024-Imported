// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class AutoShoot extends Command {
  CommandSwerveDrivetrain swerve;
  Shooter m_shooter;
  AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  SwerveRequest.FieldCentric drive;
  Camera camera;
  Intake intake;
  /** Creates a new AutoShoot. */
  public AutoShoot(CommandSwerveDrivetrain swerve, Shooter shooter, Camera camera, Intake intake, SwerveRequest.FieldCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.m_shooter = shooter;
    this.drive = drive;
    this.camera = camera;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  public double targetAll(int tagId) {
    double[] targetAngles = getTargetTagAngles(tagId);
    m_shooter.setTargetAngle(targetAngles[0]);
    double targetRotation = camera.rotateToTag(tagId);
    return targetRotation;
  }

  public double targetAll(int tagId, Supplier<Boolean> yButton) {
    double[] targetAngles = getTargetTagAngles(tagId);
    if (yButton.get()) {
      m_shooter.setTargetAngle(218.5);
    } else {
      m_shooter.setTargetAngle(targetAngles[0]);
      if (Math.abs(m_shooter.targetAngle - m_shooter.getPivotAngle()) < 0.3) {
        intake.feedMotorPower(0.5);
      }
    }
    double targetRotation = camera.rotateToTag(tagId);
    return targetRotation;
  }

  public double[] getTargetTagAngles(int tagId) {
    double botX = swerve.getOdometry().getEstimatedPosition().getX();
    double botY = swerve.getOdometry().getEstimatedPosition().getY();

    double tagX = fieldLayout.getTagPose(tagId).get().getX();
    double tagY = fieldLayout.getTagPose(tagId).get().getY();
    double relX = botX - tagX;
    double relY = botY - tagY;

    double target = Math.toDegrees(Math.atan2(relY, relX));
    SmartDashboard.putNumber("Target Robot Angle", target);
    SmartDashboard.putNumber("Current Robot Angle",
        swerve.getOdometry().getEstimatedPosition().getRotation().getDegrees());
    relX = Math.abs(relX);
    relY = Math.abs(relY);
    double distance = Math.sqrt((relX * relX) + (relY * relY)) - 0.38;
    if (distance < 2) {
      m_shooter.velocityRPM = 3000 * 0.85;
    } else {
      m_shooter.velocityRPM = 3000;
    }
    SmartDashboard.putNumber("Distance from target April Tag", distance);

    // double angle = 0.8061 * Math.pow(distance, 2) - 9.8159 * distance + 221.59;
    // This was alpha code
    double angle = 2.06331 * Math.pow(distance, 2) - 18.2605 * distance + 235.563;
    return new double[] { angle, target };
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.feedMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
