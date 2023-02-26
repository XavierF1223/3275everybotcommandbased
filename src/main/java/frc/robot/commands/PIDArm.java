// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class PIDArm extends CommandBase {
  /** Creates a new PIDArm. */
  private PIDController m_ArmPidController;
  private final Arm m_Arm;
  private double setpoint;

  public PIDArm(Arm m_Arm, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = m_Arm;
    this.setpoint = setpoint;

    m_ArmPidController = new PIDController(
      ArmConstants.armP, 
      ArmConstants.armI, 
      ArmConstants.armD);

    m_ArmPidController.setTolerance(ArmConstants.armTol);

    addRequirements(m_Arm);
  }

  public void setpoint(double setpoint){
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmPidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedforward = ArmConstants.armFF;
    double speed = m_ArmPidController.calculate(m_Arm.getEncoderRevs(), setpoint);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    speed = (speed > 1 ) ? 1.0 : speed;
    speed = (speed < -1 ) ? -1 : speed; 
    m_Arm.setMotor(speed * ArmConstants.armPower);
    SmartDashboard.putNumber("Arm Output:", speed);
    SmartDashboard.putNumber("Arm Setpoint:", setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Try return Math.abs(drive.totalDistance()) >= encoderSetpoint; if it doesnt work
    if (m_ArmPidController.atSetpoint())
      return true;
    else
      return false;
  }
}
