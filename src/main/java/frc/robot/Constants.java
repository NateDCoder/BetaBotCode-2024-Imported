// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static double NathanSpeed = 2;
  public static double MAX_SPEED = 5.2;
  public static PIDFF_CONSTANTS shooterPID = new PIDFF_CONSTANTS(6e-5, 0, 0, 0.000180);
  public static PIDFF_CONSTANTS ShooterAnglePID = new PIDFF_CONSTANTS(0.0, 0, 0, 0.0001);

  public static final int BOTTOM_SHOOTER_MOTOR_ID = 41;
  public static final int TOP_SHOOTER_MOTOR_ID = 42;
  public static final int FEED_MOTOR_ID = 43;
  public static final int PIVOT_MOTOR_ID = 44;
  public static final double MAX_PIVOT_POWER = 1;

  public static final int INTAKE_MOTOR_ID =      52;
  public static final int GUIDE_RIGHT_MOTOR_ID = 53;
  public static final int GUIDE_LEFT_MOTOR_ID =  54;

  public static final int CLIMB_LEFT_MOTOR_ID = 62;
  public static final int CLIMB_RIGHT_MOTOR_ID = 61;

  public static final double INTAKE_POWER = 0.4;

  public static final double PIVOT_ANGLE_OFFSET = -84;
  public static final double APRIL_TAG_OFFSET = 1.01237005816; //offset before 2/21/24 5:58 PM: 1.02851778742
  public static final int TEAMSWITCH_ID = 2;


  public static class PIDFF_CONSTANTS {
    public double p, i, d, ff;

    PIDFF_CONSTANTS(double p, double i, double d, double ff) {
      this.p = p;
      this.i = i;
      this.d = d;
      this.ff = ff;
    }

    public double getP() {
      return p;
    }

    public double getI() {
      return i;
    }

    public double getD() {
      return d;
    }

    public double getFF() {
      return ff;
    }
  }
}
