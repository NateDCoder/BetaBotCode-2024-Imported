// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  //61 right 62 left
  CANSparkMax  climbLeftMotor   = new CANSparkMax(Constants.CLIMB_LEFT_MOTOR_ID, MotorType.kBrushless); //ID62
  CANSparkMax  climbRightMotor   = new CANSparkMax(Constants.CLIMB_RIGHT_MOTOR_ID, MotorType.kBrushless); //ID61


  /** Creates a new Climber. */
  public Climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setLeftClimb(double power){
    climbLeftMotor.set(power);
  }
  public void setRightClimb(double power){
    climbRightMotor.set(power);
  }
}
