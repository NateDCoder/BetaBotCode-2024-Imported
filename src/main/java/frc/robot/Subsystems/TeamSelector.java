// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TeamSelector extends SubsystemBase {
  /** Creates a new TeamSelector. */
  static DigitalInput teamSwitch = new DigitalInput(Constants.TEAMSWITCH_ID);
  static String sendSwitchData;
  public TeamSelector() {}

  @Override
  public void periodic() {
    TeamSelector.getTeamColor();
    // This method will be called once per scheduler run
  }
  // this method will get the switch's boolean and then return it and print whether its red or blue.
  public static boolean getTeamColor() {
    sendSwitchData = !teamSwitch.get()? "Red" : "Blue";
    SmartDashboard.putString("Which Team", sendSwitchData);
    return !teamSwitch.get(); // True if Red, False if Blue
  }
}
