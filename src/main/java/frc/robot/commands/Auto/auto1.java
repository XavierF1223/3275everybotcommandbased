// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Arm.PIDArm;
import frc.robot.commands.Drivetrain.DriveDistance;
import frc.robot.commands.Intake.IntakeCone;
import frc.robot.subsystems.*;

public class auto1 extends CommandBase {
  /** Creates a new auto1. */
  Drivetrain m_Drivetrain;
  Intake m_Intake;
  Arm m_Arm;
  double distance;
  Timer m_tTimer;

  public auto1(Drivetrain m_Drivetrain, Intake m_Intake, Arm m_Arm, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Drivetrain = m_Drivetrain;
    this.m_Intake = m_Intake;
    this.m_Arm = m_Arm;
    this.distance = distance;
  }
  double autoStart;
  double timeElapsed;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoStart = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timeElapsed = Timer.getFPGATimestamp() - autoStart;
    if (timeElapsed < 0){
      new PIDArm(m_Arm, ArmConstants.armSetTopGoal, ArmConstants.armP2);
    }
    else if (timeElapsed < 3){
      new IntakeCone(m_Intake, IntakeConstants.intakePowerCone);
    }
    else if (timeElapsed < 4){
      new IntakeCone(m_Intake, 0);
      new PIDArm(m_Arm, ArmConstants.armSetStowed, ArmConstants.armP);
    }
    else if (timeElapsed < 6){
      new DriveDistance(m_Drivetrain, distance, -0.15);
    }
    else{
      m_Drivetrain.setMotors(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
