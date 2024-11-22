// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double MAX_SPEED = 4.5;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double LEFT_X_DEADBAND = 0.2;
    public static final double LEFT_Y_DEADBAND = 0.2;
    public static final double RIGHT_X_DEADBAND = 0.2;
  }

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.00023, 0.0000002,1);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.0020645, 0, 0);
  }
}
