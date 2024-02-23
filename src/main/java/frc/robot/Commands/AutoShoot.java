// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Shooter;

public class AutoShoot extends Command {
  CommandSwerveDrivetrain swerve;
  Shooter m_shooter;
  AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  SwerveRequest.FieldCentric drive;
  Camera camera;
  /** Creates a new AutoShoot. */
  public AutoShoot(CommandSwerveDrivetrain swerve, Shooter shooter, Camera camera, SwerveRequest.FieldCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.m_shooter = shooter;
    this.drive = drive;
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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

  public double[] getTargetTagAngles(int tagId) {
    double botX = swerve.getOdometry().getEstimatedPosition().getX();
    double botY = swerve.getOdometry().getEstimatedPosition().getY();

    double tagX = fieldLayout.getTagPose(tagId).get().getX();
    double tagY = fieldLayout.getTagPose(tagId).get().getY();
    double relX = botX - tagX;
    double relY = botY - tagY;

    double target = Math.toDegrees(Math.atan2(relY, relX));
    SmartDashboard.putNumber("Target Robot Angle", target);
    SmartDashboard.putNumber("Current Robot Angle", swerve.getOdometry().getEstimatedPosition().getRotation().getDegrees());
    relX = Math.abs(relX);
    relY = Math.abs(relY);
    double distance = Math.sqrt((relX * relX) + (relY * relY)) - 0.35;

    double angle = 0.8061 * Math.pow(distance, 2) - 9.8159 * distance + 221.59;
    return new double[]{angle, target};
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
