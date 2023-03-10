// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class neutralMode extends CommandBase {
  private final Drivetrain m_Drive;
  private final NeutralMode mode;


  public neutralMode(Drivetrain m_Drive, NeutralMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Drive = m_Drive;
    this.mode = mode;
    addRequirements(m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drive.neutralMode(mode);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    /** 
    if (mode == NeutralMode.Brake){
      m_Drive.neutralMode(NeutralMode.Coast);
      */
    }
  }
