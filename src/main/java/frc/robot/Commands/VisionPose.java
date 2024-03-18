// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Camera;

// This class will constantly updating the position if it see 2 april tags -Nathan 1/27/24
public class VisionPose extends Command {
  /** Creates a new VisionPose. */
  CommandSwerveDrivetrain swerve;
  Camera camera;

  public VisionPose(CommandSwerveDrivetrain swerve, Camera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.camera = camera;

    addRequirements(camera);
  }

  public double getVisionDeviation(Optional<EstimatedRobotPose> result) {
    if (!result.isPresent()) {
      return 0;
    }

    EstimatedRobotPose camPose = result.get();

    return swerve.getOdometry().getEstimatedPosition().minus(camPose.estimatedPose.toPose2d()).getTranslation()
        .getNorm();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Just printing stuff for debugging purposes
    Optional<EstimatedRobotPose> resultLeft = camera
        .getLeftEstimatedGlobalPose(swerve.getOdometry().getEstimatedPosition());
    Optional<EstimatedRobotPose> resultRight = camera
        .getRightEstimatedGlobalPose(swerve.getOdometry().getEstimatedPosition());
    if (!resultLeft.isPresent() && !resultRight.isPresent()) {
      return;
    }
    int totalTags = camera.getLeftLatestResult().targets.size() + camera.getRightLatestResult().targets.size();
    SmartDashboard.putNumber("BotPosX", swerve.getOdometry().getEstimatedPosition().getX());
    SmartDashboard.putNumber("BotPosY", swerve.getOdometry().getEstimatedPosition().getY());
    SmartDashboard.putNumber("BotPosHeading", swerve.getOdometry().getEstimatedPosition().getRotation().getDegrees());
    if (resultLeft.isPresent()) {
      EstimatedRobotPose camPoseLeft = resultLeft.get();
      if (isValidPose(camPoseLeft.estimatedPose.toPose2d(),
          swerve.getOdometry().getEstimatedPosition())
          && camera.getLeftLatestResult().targets.size() > 1) {
        SmartDashboard.putNumber("Left X", camPoseLeft.estimatedPose.toPose2d().getX());// * 0.96432039823);
        SmartDashboard.putNumber("Left Y", camPoseLeft.estimatedPose.toPose2d().getY());
        SmartDashboard.putNumber("Left Rotation", camPoseLeft.estimatedPose.toPose2d().getY());
        swerve.setOdometryVisionMeasurement(new Pose2d(camPoseLeft.estimatedPose.toPose2d().getX(),// * 0.96432039823,
            camPoseLeft.estimatedPose.toPose2d().getY(),
            camPoseLeft.estimatedPose.toPose2d().getRotation()),
            camPoseLeft.timestampSeconds);
      }
      if (swerve.getOdometry().getEstimatedPosition().getX() < 1
          && swerve.getOdometry().getEstimatedPosition().getY() < 1
          && camera.getLeftLatestResult().targets.size() > 1) {
        swerve.seedFieldRelative(camPoseLeft.estimatedPose.toPose2d());
      }
    }
    if (resultRight.isPresent()) {
      EstimatedRobotPose camPoseRight = resultRight.get();
      if (isValidPose(camPoseRight.estimatedPose.toPose2d(), swerve.getOdometry().getEstimatedPosition())
          && camera.getRightLatestResult().targets.size() > 1) {
        swerve.setOdometryVisionMeasurement(new Pose2d(camPoseRight.estimatedPose.toPose2d().getX(),
            camPoseRight.estimatedPose.toPose2d().getY(), //* 1.01189,
            camPoseRight.estimatedPose.toPose2d().getRotation()), camPoseRight.timestampSeconds);
        SmartDashboard.putNumber("Right X", camPoseRight.estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("Right Y", camPoseRight.estimatedPose.toPose2d().getY()); //* 1.01189);
        SmartDashboard.putNumber("RIght Rotation", camPoseRight.estimatedPose.toPose2d().getRotation().getDegrees());
      }
      if (swerve.getOdometry().getEstimatedPosition().getX() < 1
          && swerve.getOdometry().getEstimatedPosition().getY() < 1
          && camera.getRightLatestResult().targets.size() > 1) {
        swerve.seedFieldRelative(camPoseRight.estimatedPose.toPose2d());
      }
    }
  }

  public boolean isValidPose(Pose2d VisionPose, Pose2d BotPose) {
    return (VisionPose.getX() != 0 &&
        VisionPose.getY() != 0 &&
        VisionPose.getX() != 0 &&
        VisionPose.getY() != 0 &&
        Math.abs(BotPose.getX() - VisionPose.getX()) < 1.0 &&
        Math.abs(BotPose.getY() - VisionPose.getY()) < 1.0);
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
