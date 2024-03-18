// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.io.IOError;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Commands.VisionPose;
import frc.robot.generated.TunerConstants;

// Handle the camera and vision pipeline 1/27/24 -Nathan
public class Camera extends SubsystemBase {
  // Nathan instead of doing -20, I put in 340 which is 360 - 20...
  private static final Transform3d BACK_RIGHT_CAMERA_TO_CENTER = new Transform3d(
      new Translation3d(-0.04, -.22, 0.20),
      new Rotation3d(Math.toDegrees(0),
          Math.toRadians(-33), Math.toRadians(200)));
  private static final Transform3d BACK_LEFT_CAMERA_TO_CENTER = new Transform3d(
      new Translation3d(-0.34, .22, 0.20),
      new Rotation3d(Math.toDegrees(0),
          Math.toRadians(-33), Math.toRadians(160)));
  // PhotonCamera camera;

  // PhotonPoseEstimator photonPoseEstimator;

  private PhotonCamera camRight;
  private PhotonCamera camLeft;
  public final PhotonPoseEstimator photonPoseEstimatorRight;
  public final PhotonPoseEstimator photonPoseEstimatorLeft;
  CommandSwerveDrivetrain drivetrain;

  /** Creates a new Camera. */
  public Camera(CommandSwerveDrivetrain drivetrain) {
    AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    camLeft = new PhotonCamera("Back_Left_Camera");// Have to change these names eventually
    camRight = new PhotonCamera("Back_Right_Camera");
    // if(camLeft.getLatestResult().getTargets().size() > 1) {
    // camRight = new PhotonCamera("Back_Left_Camera");
    // }else {
    // camRight = new PhotonCamera("Back_Right_Camera");
    // camLeft = new PhotonCamera("Back_Left_Camera");
    // }
    photonPoseEstimatorRight = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camRight,
        BACK_RIGHT_CAMERA_TO_CENTER);
    photonPoseEstimatorLeft = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camLeft,
        BACK_LEFT_CAMERA_TO_CENTER);

    photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    this.drivetrain = drivetrain;
  }

  public PhotonPipelineResult getLeftLatestResult() {
    return camLeft.getLatestResult();
  }

  public PhotonPipelineResult getRightLatestResult() {
    return camRight.getLatestResult();
  }

  public boolean checkTargetExistence(PhotonPipelineResult result) {
    return result.hasTargets();
  }

  public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
    return result.getTargets();
  }

  public PhotonTrackedTarget getBesTarget(PhotonPipelineResult result) {
    return result.getBestTarget();
  }

  public Transform3d getTag3D(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
  }

  public double getLeftLatency() {
    return getLeftLatestResult().getLatencyMillis() / 1000.0;
  }

  public Optional<EstimatedRobotPose> getLeftEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimatorLeft.setReferencePose(prevEstimatedRobotPose);
    SmartDashboard.putNumber("Amount of tags detected Left", getLeftLatestResult().targets.size());
    return photonPoseEstimatorLeft.update();
  }

  public Optional<EstimatedRobotPose> getRightEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimatorRight.setReferencePose(prevEstimatedRobotPose);
    SmartDashboard.putNumber("Amount of tags detected Right", getRightLatestResult().targets.size());
    return photonPoseEstimatorRight.update();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new VisionPose(drivetrain, this));
  }
}
