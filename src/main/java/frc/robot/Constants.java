// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double stickDeadband = .1;
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorCANIds { 
    public static final int FRONT_LEFT = 1;
    public static final int REAR_LEFT = 2;
    public static final int REAR_RIGHT = 3;
    public static final int FRONT_RIGHT = 4;
  }

  public static final double wheelDiameter = Units.inchesToMeters(4.0);
  public static final double wheelCircumference = wheelDiameter * Math.PI;

  public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1 // TODO:: MEASURE ON ROBOT

}
