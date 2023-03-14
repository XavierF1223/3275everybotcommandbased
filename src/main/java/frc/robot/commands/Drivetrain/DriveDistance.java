// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {

  private final Drivetrain m_Drive;
  private final double distance;
  private double encoderSetpoint;
  private double driveDistSpd;

  public DriveDistance(Drivetrain m_Drive, double distance, double driveDistSpd) {
    this.m_Drive = m_Drive;
    this.distance = distance;
    this.driveDistSpd = driveDistSpd;
    addRequirements(m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderSetpoint = m_Drive.totalDistance() + distance;
    System.out.println("DriveDistance started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drive.setMotors(driveDistSpd, driveDistSpd);
    SmartDashboard.putNumber("encoder total distance", m_Drive.totalDistance());
    SmartDashboard.putNumber("encoder setpoint distance", encoderSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drive.setMotors(0, 0);
    System.out.println("DriveDistance ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_Drive.totalDistance()) >= encoderSetpoint;
  }
}
