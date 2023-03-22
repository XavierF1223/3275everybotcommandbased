// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Arm.PIDArm;
import frc.robot.commands.Drivetrain.DriveDistance;
import frc.robot.commands.Intake.IntakeCone;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;


public class ConeDrive extends SequentialCommandGroup {
  Drivetrain m_Drivetrain;
  Intake m_Intake;
  Arm m_Arm;

  public ConeDrive(Drivetrain m_Drivetrain, Intake m_Intake, Arm m_Arm, double distance) {
    this.m_Drivetrain = m_Drivetrain;
    this.m_Intake = m_Intake;
    this.m_Arm = m_Arm;

    addCommands(
      new PIDArm(m_Arm, ArmConstants.armSetMidGoal, ArmConstants.armP2).until(()-> m_Arm.isTol()),
      Commands.waitSeconds(1),
      Commands.runOnce(()->m_Intake.setMotor(0.6), m_Intake),
      Commands.waitSeconds(1),
      Commands.runOnce(()->m_Intake.setMotor(0), m_Intake),
      Commands.runOnce(()->m_Arm.setMotor(-0.10), m_Arm),
      Commands.waitSeconds(3),
      Commands.runOnce(()->m_Arm.setMotor(0), m_Arm),
      new DriveDistance(m_Drivetrain, distance, -0.15),
      Commands.waitSeconds(8)
    
      //Commands.run(()->m_Drivetrain.setMotors(-0.15, -0.15), m_Drivetrain).until(()-> m_Drivetrain.isTol(2.15))
    );
    
  }
}
