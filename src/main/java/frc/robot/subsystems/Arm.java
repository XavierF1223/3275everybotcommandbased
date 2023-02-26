// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Arm extends SubsystemBase {
  TalonFX armFx1 = new TalonFX(ArmConstants.ArmMotorLeft);
  TalonFX armFx2 = new TalonFX(ArmConstants.ArmMotorRight);
  /** Creates a new Arm. */
  public Arm() {
    armFx1.configFactoryDefault();
    armFx2.configFactoryDefault();
    armFx1.setNeutralMode(NeutralMode.Brake);
    armFx2.setNeutralMode(NeutralMode.Brake);
    armFx2.follow(armFx1);

  }

  public double getEncoderRevs(){
    // 2048 ticks per revolution of shaft, shaft on 60:1 gearbox, average of BOTH motors
    return ( (Math.abs(armFx1.getSelectedSensorPosition()) + 
              Math.abs(armFx2.getSelectedSensorPosition())) /2
              /2048 /60 );
  }

  public void setMotor(double speed){
    armFx1.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    armFx1.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder Value", getEncoderRevs());
  }
}
