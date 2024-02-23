// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TeamSelector extends SubsystemBase {
  /** Creates a new TeamSelector. */
  // static DigitalInput teamSwitch = new DigitalInput(0);
  public TeamSelector() {}

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Color Indicator", teamSwitch.get());
    // This method will be called once per scheduler run
  }

  public static boolean getTeamColor() {
    return false; // True if Red, False if Blue
  }
}
