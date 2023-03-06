// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
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
  //private double P;

  public PIDArm(Arm m_Arm, double setpoint, double P) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = m_Arm;
    this.setpoint = setpoint;
    //this.P = P;

    m_ArmPidController = new PIDController(
      P, 
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
    m_Arm.setMotor(MathUtil.clamp(speed, -0.25, 0.25));
    SmartDashboard.putNumber("Arm Output:", speed);
    SmartDashboard.putNumber("Arm Setpoint:", setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.setMotor(0);
    m_Arm.setMotorHold(25);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_ArmPidController.atSetpoint())
      return true;
    return false;
  }
}
