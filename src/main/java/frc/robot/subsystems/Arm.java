// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Arm extends SubsystemBase {
  TalonFX armFx = new TalonFX(ArmConstants.ArmMotor);
  /** Creates a new Arm. */
  public Arm() {
    armFx.setNeutralMode(NeutralMode.Brake);
  }

  public double getEncoderRevs(){
    // 4096 ticks per revolution of shaft, shaft on 75:1 gearbox
    return armFx.getSelectedSensorPosition() /4096 /75;
  }

  public void setMotor(double speed){
    armFx.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
