// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends SubsystemBase {
  static TalonSRX intakeSRX = new TalonSRX(IntakeConstants.IntakeMotor);

  public Intake() {
    intakeSRX.configFactoryDefault();
    intakeSRX.setNeutralMode(NeutralMode.Brake);
  }

  public void setMotor(double speed){
    intakeSRX.set(ControlMode.PercentOutput, speed);
  }
  public void stop(){
    intakeSRX.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
