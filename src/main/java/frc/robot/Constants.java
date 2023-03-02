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
  //MOTOR CAN IDS
  public static final int FL = 52;
  public static final int FR = 50;
  public static final int RL = 53;
  public static final int RR = 51;
  //DRIVE CONSTANTS
  public static final double Deadband = 0.1;
  public static double MaxDriveOutput = 0.9;
  public static double MaxTurnOutput = 0.9;
  //MISC DRIVEBASE CONSTANTS
  public static final int Front_Angle_Offset = 0;
  public static final double driveDistSpd = 0.15;

}

public static class ArmConstants {
  //MOTOR CAN ID
  public static final int ArmMotorLeft = 5;
  public static final int ArmMotorRight = 7;
  //ARM MOTOR OUTPUT IN PERCENTOUTPUT
  public static final double armPower = 0.15;
  //ARM PID CONSTANTS
  public static final double armFF = 0;//NOT NEEDED(?)
  public static final double armP = 0.00005;//ARM STOW/MID EXTENDED
  public static final double armP2 = 0.00007;//ARM FULL EXTENDED
  public static final double armI = 0;// NOT NEEDED
  public static final double armD = 0;// NOT NEEDED
  public static final double armTol = 0.01;
  public static final int armSetStowed = -1024;//close to 0 but not 0 to avoid OVERDRIVE
  public static final int armSetMidGoal = -30000;
  public static final int armSetTopGoal = -53000;

}

public static class IntakeConstants {
  //MOTOR CAN ID
  public static final int IntakeMotor = 6;
  //MOTOR INTAKE POWERS
  public static final double intakePowerConeIn = 0.9;
  public static final double intakePowerCubeIn = 0.65;

}

public static class AutoConstants {
  public static final double autoDistance = 1; //SET
}
}
