// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake extends SubsystemBase {
  static TalonFX intakeFx = new TalonFX(IntakeConstants.IntakeMotor);

  public Intake() {
    intakeFx.configFactoryDefault();
    intakeFx.setNeutralMode(NeutralMode.Coast);
  }

  public static void setMotor(double speed){
    intakeFx.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
