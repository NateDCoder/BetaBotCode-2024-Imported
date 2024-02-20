// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
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

    return swerve.getOdometry().getEstimatedPosition().minus(camPose.estimatedPose.toPose2d()).getTranslation().getNorm();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  Pose2d estimatedBotPose;
  Pose2d camPose2d;
  Pose3d camPose3d;
  double botX, botY;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(estimatedBotPose);
    if (!result.isPresent()) { return; }
    EstimatedRobotPose camPose = result.get();

    estimatedBotPose = swerve.getOdometry().getEstimatedPosition();
    camPose2d = camPose.estimatedPose.toPose2d();
    camPose3d = camPose.estimatedPose;

    botX = estimatedBotPose.getX(); botY = estimatedBotPose.getY();

    SmartDashboard.putNumber("BotX", botX);
    SmartDashboard.putNumber("BotY", botY);
    SmartDashboard.putNumber("BotHeading", estimatedBotPose.getRotation().getDegrees());

    SmartDashboard.putBoolean("Camera Targets Present", result.isPresent());
    // SmartDashboard.putNumber("BotPosHeading",
    // camPose.estimatedPose.toPose2d().getRotation().getDegrees());
    if ((camPose3d.getX() != 0 && camPose3d.getY() != 0 &&
    /*
     * Only update if the camera pose is within 1 meter of the estimated position.
     * This will hopefully remove values that are not realistic.
     */
        Math.abs(botX - camPose3d.getX()) < 1.0 && Math.abs(botY - camPose3d.getY()) < 1.0 && 
        camera.getLatestResult().targets.size() > 1)) {
      swerve.setOdometryVisionMeasurement(new Pose2d(camPose2d.getX() * Constants.APRIL_TAG_OFFSET,
      camPose2d.getY(), camPose2d.getRotation()), camPose.timestampSeconds);
      SmartDashboard.putNumber("AprilTagX", camPose2d.getX() * Constants.APRIL_TAG_OFFSET);
      SmartDashboard.putNumber("AprilTagY", camPose2d.getY());
      SmartDashboard.putNumber("AprilTagHeading", camPose2d.getRotation().getDegrees());
    } else if ((Math.abs(swerve.getOdometry().getEstimatedPosition().getX()) < 1 &&
        Math.abs(swerve.getOdometry().getEstimatedPosition().getY()) < 1)) {
      swerve.seedFieldRelative(new Pose2d(camPose2d.getX() * Constants.APRIL_TAG_OFFSET,
          camPose2d.getY(), camPose2d.getRotation()));

    }
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
