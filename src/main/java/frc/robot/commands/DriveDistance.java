// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {

  private final Drivetrain drive;
  private final double distance;
  private double encoderSetpoint;

  public DriveDistance(Drivetrain drive, double distance) {
    this.drive = drive;
    this.distance = distance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderSetpoint = drive.totalDistance() + distance;
    System.out.println("DriveDistance started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setMotors(DriveConstants.driveDistSpd , DriveConstants.driveDistSpd);
    SmartDashboard.putNumber("encoder total distance", drive.totalDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setMotors(0, 0);
    System.out.println("DriveDistance ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drive.totalDistance()) >= encoderSetpoint;
  }
}
