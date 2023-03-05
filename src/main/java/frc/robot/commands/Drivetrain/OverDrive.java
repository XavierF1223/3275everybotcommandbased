// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OverDrive extends CommandBase {

  public OverDrive() {
  }

  @Override
  public void initialize() {
    DriveConstants.MaxDriveOutput = 0.8;
    DriveConstants.MaxTurnOutput = 0.8;
  }

  @Override
  public void end(boolean interrupted) {
    DriveConstants.MaxDriveOutput = 0.5;
    DriveConstants.MaxTurnOutput = 0.5;
  }
}

