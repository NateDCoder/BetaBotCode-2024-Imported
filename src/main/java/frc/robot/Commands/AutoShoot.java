// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class AutoShoot extends Command {
  double formulaVelocity;
  CommandSwerveDrivetrain swerve;
  Shooter m_shooter;
  AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  PIDController turnPID = new PIDController(0.3, 0, 0.09e-1);
  SwerveRequest.FieldCentric drive;
  Camera camera;
  Intake intake;

  /** Creates a new AutoShoot. */
  public AutoShoot(CommandSwerveDrivetrain swerve, Shooter shooter, Camera camera, Intake intake,
      SwerveRequest.FieldCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.m_shooter = shooter;
    this.drive = drive;
    this.camera = camera;
    this.intake = intake;

    turnPID.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public double targetAuton(int tagId) {
    m_shooter.setTargetAngle(getTargetTagAngles(tagId)[0]);
    return rotateToTag(tagId)[0];
  }

  public double targetAll(int tagId, Supplier<CommandXboxController> controller) {
    double botRotation = swerve.getOdometry().getEstimatedPosition().getRotation().getDegrees();
    double[] targetAngles = getTargetTagAngles(tagId);
    double[] targetRotation = rotateToTag(tagId);

    double rotationError = Math.abs(botRotation) - Math.abs(targetRotation[1]);

    if (controller.get().y().getAsBoolean()) { m_shooter.setTargetAngle(218.5); }
    else if (controller.get().leftBumper().getAsBoolean()) { m_shooter.setTargetAngle(204); }
    else {
      m_shooter.setTargetAngle(targetAngles[0]);
      SmartDashboard.putNumber ("Rotation Error", rotationError);
      if (Math.abs(m_shooter.targetAngle - m_shooter.getPivotAngle()) < 0.3 &&
      (rotationError < 0.75 && rotationError > -0.75)) { intake.feedMotorPower(0.5); }
    }

    return targetRotation[0];
  }

  public double[] rotateToTag(int tagId) {
    double botX = swerve.getOdometry().getEstimatedPosition().getX();
    double botY = swerve.getOdometry().getEstimatedPosition().getY();
    double botRotation = swerve.getOdometry().getEstimatedPosition().getRotation().getDegrees();

    double tagX = fieldLayout.getTagPose(tagId).get().getX();
    double tagY = fieldLayout.getTagPose(tagId).get().getY();
    double relX = botX - tagX;
    double relY = botY - tagY;

    double target = Math.toDegrees(Math.atan2(relY, relX));

    return new double[] {turnPID.calculate(botRotation, target), target};
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

    //x = distance
    //f(x) = 83.690131x^3 - 651.40176x^2 + 1892.6791x + 1266.9023

    formulaVelocity = 83.690131*Math.pow(distance,3)-651.40176*Math.pow(distance,2)+1892.6791*distance+1266.9023;
    if (formulaVelocity >= 5000){
      m_shooter.velocityRPM = 5000;
    } else if (formulaVelocity <= 1300) {
      m_shooter.velocityRPM = 1300;
    } else {
      m_shooter.velocityRPM = formulaVelocity;
    }
    
    SmartDashboard.putNumber("Formula Velocity",  formulaVelocity);
    SmartDashboard.putNumber("Distance from target April Tag", distance);
    SmartDashboard.putNumber("Distance from target April Tag X", relX);
    SmartDashboard.putNumber("Distance from target April Tag Y", relY);
    // double angle = 0.8061 * Math.pow(distance, 2) - 9.8159 * distance + 221.59;
    // This was alpha code
    // double angle = 2.06331 * Math.pow(distance, 2) - 18.2605 * distance + 235.563;
    // Jackson event formula

    double angle = 0.98515735 * Math.pow(distance, 2) - 12.955594 * distance + 231.53526 + 0.1;

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
