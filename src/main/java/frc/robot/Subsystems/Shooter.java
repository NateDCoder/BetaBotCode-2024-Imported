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
import frc.robot.Constants.PIDFF_CONSTANTS;

public class Shooter extends SubsystemBase {

  CANSparkMax pivot = new CANSparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
  CANSparkFlex bottomShooter = new CANSparkFlex(Constants.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
  CANSparkFlex topShooter = new CANSparkFlex(Constants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
  DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  SparkPIDController bottomShooterPID, topShooterPID;
  PIDController pivotPID;

  public RelativeEncoder shooterTopEncoder, shooterBottomEncoder;
  public double velocityRPM, targetAngle = 190;
  public double p = 6e-5, i = 0, d = 0, pivotFF = 0.000180;
  public double ffPower = 0.024;
  public boolean enableShooter = false;

  // public double tempTargetAngle = 190;

  /** Creates a new ShooterSubsystem. */
  public Shooter() {
    topShooterPID = topShooter.getPIDController();
    bottomShooterPID = bottomShooter.getPIDController();

    // Oringinal values set by Nathan.
    // pivotPID = new PIDController(0.02, 0.0, 0.0);
    pivotPID = new PIDController(0.02, 0.00001, 0.000001);

    // pivotPID.setFeedbackDevice(pivotEncoder);
    shooterTopEncoder = topShooter.getEncoder();
    shooterBottomEncoder = bottomShooter.getEncoder();

    velocityRPM = 3000; // Made this number up
    configureShooterPID(topShooterPID, Constants.shooterPID);
    configureShooterPID(bottomShooterPID, Constants.shooterPID);

    // pivotPID.setP(p);
    // pivotPID.setI(i);
    // pivotPID.setD(d);

    // SmartDashboard.putNumber("P Angle", p);
    // SmartDashboard.putNumber("I Angle", i);
    // SmartDashboard.putNumber("D Angle", d);
    // SmartDashboard.putNumber("FF Angle", pivotFF);

    SmartDashboard.putNumber("Velocity", velocityRPM);
    SmartDashboard.putNumber("Target Angle", targetAngle);
    enableShooter = false;
  }

  public void setTargetAngle(double tAngle) {
    targetAngle = tAngle;
  }

  public void configurePID(SparkPIDController motorPID, double p, double i, double d, double ff) {
    motorPID.setP(p);
    motorPID.setI(i);
    motorPID.setD(d);
    motorPID.setFF(ff);
    motorPID.setOutputRange(-1, 1);
  }

  public void configureShooterPID(SparkPIDController motorPID, PIDFF_CONSTANTS shooterConstants) {
    configurePID(motorPID, shooterConstants.getP(), shooterConstants.getI(), shooterConstants.getD(),
        shooterConstants.getFF());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler
    SmartDashboard.putNumber("Top Speed", shooterTopEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Speed", shooterBottomEncoder.getVelocity());
    SmartDashboard.putNumber("Pivot Encoder", getPivotAngle());
  }

  public void setShooterVelocity() {
    // topShooterPID.setReference(velocityRPM, CANSparkFlex.ControlType.kVelocity);
    // bottomShooterPID.setReference(-velocityRPM, CANSparkFlex.ControlType.kVelocity);

    topShooterPID.setReference(velocityRPM, CANSparkFlex.ControlType.kVelocity);
    bottomShooterPID.setReference(-velocityRPM * 0.8, CANSparkFlex.ControlType.kVelocity);
    SmartDashboard.putNumber("Target", velocityRPM);
    SmartDashboard.putNumber("Actual Top", shooterTopEncoder.getVelocity());
    SmartDashboard.putNumber("Actual Bottom", shooterBottomEncoder.getVelocity());
    SmartDashboard.putNumber("Difference of Shooter",
        shooterTopEncoder.getVelocity() - Math.abs(shooterBottomEncoder.getVelocity()));

  }

  public void stopShooter() {
    // topShooterPID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
    // bottomShooterPID.setReference(0.0, CANSparkFlex.ControlType.kVelocity);
    topShooter.set(0);
    bottomShooter.set(0);
  }

  public void setPivotAngle() {
    SmartDashboard.putNumber("Target Angle", targetAngle);
    if (targetAngle < 185 || targetAngle > 223 || getPivotAngle() > 223) {
    pivot.set(0);
    SmartDashboard.putNumber("Targeting an invalid value", targetAngle);
    return;
    }
    // if (tempTargetAngle < 185 || tempTargetAngle > 223 || getPivotAngle() > 223) {
    //   pivot.set(0);
    //   SmartDashboard.putNumber("Targeting an invalid value", targetAngle);
    //   return;
    // }

    // This is the amount of power to keep it in a given place

    double power = (pivotPID.calculate(getPivotAngle(), targetAngle) + ffPower);
    // double power = (pivotPID.calculate(getPivotAngle(), tempTargetAngle) + ffPower);

    if (getPivotAngle() < 185 && Math.signum(power) == -1) {
      SmartDashboard.putString("Trying to hit ground", "1");
      pivot.set(0);
      return;
    }

    SmartDashboard.putNumber("PID Power", Math.signum(power) * Math.min(Math.abs(power), Constants.MAX_PIVOT_POWER));
    pivot.set(Math.signum(power) * Math.min(Math.abs(power), Constants.MAX_PIVOT_POWER));
  }

  public void stopPivot() {
    pivot.set(0.0);
  }

  public double getPivotAngle() {
    double correctedAngle = pivotEncoder.getAbsolutePosition() * 360 + Constants.PIVOT_ANGLE_OFFSET;
    if (correctedAngle < 0) { correctedAngle += 360; }
    if (correctedAngle > 360) { correctedAngle %= 360; }
    return correctedAngle;
  }
}
