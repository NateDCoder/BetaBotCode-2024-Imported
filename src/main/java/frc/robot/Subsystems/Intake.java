// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax  feedMotor   = new CANSparkMax(Constants.FEED_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax  guideLeft   = new CANSparkMax(Constants.GUIDE_LEFT_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax  guideRight  = new CANSparkMax(Constants.GUIDE_RIGHT_MOTOR_ID, MotorType.kBrushless);
  CANSparkFlex intakeMotor = new CANSparkFlex(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {
  }

  public void feedMotorPower(double power) {
    feedMotor.set(power);
  }

  public void intakeMotorPower(double power) {
    intakeMotor.set(power);
    guideLeft.set(-power);
    guideRight.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
