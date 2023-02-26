// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;

public class Orchestra extends SubsystemBase {
  TalonFX []_instruments = {
    new TalonFX(DriveConstants.FL),
    new TalonFX(DriveConstants.FR),
    new TalonFX(DriveConstants.RL),
    new TalonFX(DriveConstants.RR),
    new TalonFX(ArmConstants.ArmMotorLeft),
    new TalonFX(ArmConstants.ArmMotorRight)};

  String []_songs = new String[]{
    "song1.chrp" //etc
  };
  int _songSelection = 0;
  int _timeToPlayLoops = 0;
  
  /** Creates a new Orchestra. */
  public Orchestra() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
