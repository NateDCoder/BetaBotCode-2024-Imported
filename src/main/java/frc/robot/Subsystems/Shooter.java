// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  CANSparkMax pivot = new CANSparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
  CANSparkFlex bottomShooter = new CANSparkFlex(Constants.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
  CANSparkFlex topShooter = new CANSparkFlex(Constants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
  DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  SparkPIDController bottomShooterPID, topShooterPID;
  PIDController pivotPID;
  public RelativeEncoder shooterTopEncoder, shooterBottomEncoder;
  public double velocityRPM, targetAngle = 190;
  public double p = 0.01, i = 0, d = 0, pivotFF = 0.03;

  /** Creates a new ShooterSubsystem. */
  public Shooter() {
    topShooterPID = topShooter.getPIDController();
    bottomShooterPID = bottomShooter.getPIDController();

    pivotPID = new PIDController(0.01, 0, 0);

    // pivotPID.setFeedbackDevice(pivotEncoder);
    shooterTopEncoder = topShooter.getEncoder();
    shooterBottomEncoder = bottomShooter.getEncoder();

    velocityRPM = 3000; // Made this number up
    configurePID(topShooterPID, Constants.shooterPID.getP(), Constants.shooterPID.getI(), Constants.shooterPID.getD(),
        Constants.shooterPID.getFF());
    configurePID(bottomShooterPID, Constants.shooterPID.getP(), Constants.shooterPID.getI(),
        Constants.shooterPID.getD(), Constants.shooterPID.getFF());

    // pivotPID.setP(p);
    // pivotPID.setI(i);
    // pivotPID.setD(d);

    SmartDashboard.putNumber("P Angle", p);
    SmartDashboard.putNumber("I Angle", i);
    SmartDashboard.putNumber("D Angle", d);
    SmartDashboard.putNumber("FF Angle", pivotFF);

    SmartDashboard.putNumber("Velocity", velocityRPM);
    SmartDashboard.putNumber("Target Angle", targetAngle);

  }

  public void configurePID(SparkPIDController motorPID, double p, double i, double d, double ff) {
    motorPID.setP(p);
    motorPID.setI(i);
    motorPID.setD(d);
    motorPID.setFF(ff);
    motorPID.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler
    SmartDashboard.putNumber("Pivot Encoder", getPivotAngle());
  }

  public void setShooterVelocity() {
    topShooterPID.setReference(velocityRPM, CANSparkFlex.ControlType.kVelocity);
    bottomShooterPID.setReference(-velocityRPM, CANSparkFlex.ControlType.kVelocity);

    SmartDashboard.putNumber("Target", velocityRPM);
    SmartDashboard.putNumber("Actual Top", shooterTopEncoder.getVelocity());
    SmartDashboard.putNumber("Actual Bottom", shooterBottomEncoder.getVelocity());
    SmartDashboard.putNumber("Difference of Shooter",
        shooterTopEncoder.getVelocity() - Math.abs(shooterBottomEncoder.getVelocity()));
  }

  public void stopShooter() {
    topShooterPID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
    bottomShooterPID.setReference(0.0, CANSparkFlex.ControlType.kVelocity);
  }

  public void setPivotAngle() {
    if (185 < targetAngle && targetAngle < 215) {
      // This is geometry stuff for force required to keep it in a spot
      double ffPower = 0.0000412372 * Math.pow((getPivotAngle() - 170), 2) + 0.0185154;
      SmartDashboard.putNumber("Pivot FF Power", ffPower);
      double power = -(pivotPID.calculate(getPivotAngle(), targetAngle) + ffPower);
      SmartDashboard.putNumber("PID Power", power);

      pivot.set(Math.signum(power) * Math.min(Math.abs(power), Constants.MAX_PIVOT_POWER));

    } else {
      pivot.set(0);
    }
  }

  public double getPivotAngle() {
    return (pivotEncoder.getAbsolutePosition() * 360 + Constants.PIVOT_ANGLE_OFFSET) % 360;
  }
}
