// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  boolean run;

  public DigitalInput irSensor = new DigitalInput(1);
  CANSparkMax feedMotor = new CANSparkMax(Constants.FEED_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax guideLeft = new CANSparkMax(Constants.GUIDE_LEFT_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax guideRight = new CANSparkMax(Constants.GUIDE_RIGHT_MOTOR_ID, MotorType.kBrushless);
  CANSparkFlex intakeMotor = new CANSparkFlex(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);

  boolean robotRunning;

  /** Creates a new Intake. */
  public Intake() {
    feedMotor.setIdleMode(IdleMode.kCoast);
  }

  public void feedMotorPower(double power) {
    feedMotor.set(power);
  }

  public void intakeMotorPower(double power) {
    intakeMotor.set(power);
    guideLeft.set(power);
    guideRight.set(-power);
  }

  public void autoIntake() {
    run = true;
  }

  public void updateAutoIntake() {
    if (run) {
      if (irSensor.get()) {
        feedMotor.set(0.5);
        intakeMotor.set(Constants.INTAKE_POWER);
        guideLeft.set(Constants.INTAKE_POWER);
        guideRight.set(-Constants.INTAKE_POWER);
      } else {
        feedMotor.set(0.0);
        intakeMotor.set(0.0);
        guideLeft.set(0.0);
        guideRight.set(0.0);
      }
    }
  }

  public void stopIntakeAndFeed() {
    feedMotor.set(0.0);
    intakeMotor.set(0.0);
    guideLeft.set(0.0);
    guideRight.set(0.0);
    run = false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note In Shooter", !irSensor.get());
    SmartDashboard.putBoolean("Run Teleop Intake", run);
    if (RobotState.isTeleop()) {
      updateAutoIntake();
    }
  }
}
