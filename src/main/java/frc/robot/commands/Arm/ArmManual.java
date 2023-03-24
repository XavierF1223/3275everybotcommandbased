// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmManual extends CommandBase {
  private final Arm m_Arm;
  private final double speed;

  /** Creates a new ArmManual. */
  public ArmManual(Arm m_Arm, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = m_Arm;
    this.speed = speed;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.setMotor(0);
    m_Arm.setMotorHold(ArmConstants.armHoldPowerAmps);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
