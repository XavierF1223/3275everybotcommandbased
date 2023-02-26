// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

public static class DriveConstants {
  public static final int FL = 52;
  public static final int FR = 50;
  public static final int RL = 53;
  public static final int RR = 51;
  public static final double Deadband = 0.1;
  public static double MaxDriveOutput = 0.9;
  public static double MaxTurnOutput = 0.9;
  public static final int Front_Angle_Offset = 0;
  public static final double driveDistSpd = 0.15;

}

public static class ArmConstants {
  public static final int ArmMotor = 5;
  public static final double armPower = 0.15;
  public static final double armFF = -0.05;
  public static final double armP = 0.004;
  public static final double armI = 0;
  public static final double armD = 0;
  public static final double armTol = 0.004;
  public static final double armSet1 = 10;
  public static final double armSet2 = 20;

}

public static class IntakeConstants {
  public static final int IntakeMotor = 6;
  public static final double intakePower1 = 0.9;
  public static final double intakePower2 = 0.65;

}

public static class AutoConstants {
  public static final double autoDistance = 1; //SET
}
}
