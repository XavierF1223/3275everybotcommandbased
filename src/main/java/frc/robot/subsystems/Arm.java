// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class Arm extends SubsystemBase {
  TalonFX armFx1 = new TalonFX(ArmConstants.ArmMotorLeft);
  TalonFX armFx2 = new TalonFX(ArmConstants.ArmMotorRight);
  TalonFXConfiguration armConfig = new TalonFXConfiguration();
  DutyCycleEncoder enc = new DutyCycleEncoder(0);

  /** Creates a new Arm. */
  public Arm() {
    configure();
  }
  public void configure(){
    armFx2.follow(armFx1);

    armFx1.configFactoryDefault();
    armFx2.configFactoryDefault();

    armFx1.setNeutralMode(NeutralMode.Brake);
    armFx2.setNeutralMode(NeutralMode.Brake);

    armFx1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 1));
    armFx2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 1));

    armFx2.setInverted(false);

    armConfig.slot0.kP = ArmConstants.armP;
    armConfig.slot0.kI = ArmConstants.armI;
    armConfig.slot0.kD = ArmConstants.armD;
    armConfig.slot0.allowableClosedloopError = ArmConstants.armTol;
    armConfig.slot0.closedLoopPeakOutput = ArmConstants.armPower;
    armFx1.configAllSettings(armConfig);
  }

  /** Returns average encoder ticks */
  public double getEncoderRevs(){
    //return enc.getDistance();
    return (armFx1.getSelectedSensorPosition() + 
              armFx2.getSelectedSensorPosition()) /2;
  }

  public boolean isTol(){
    return (getEncoderRevs() >= ArmConstants.armSetTopGoal-2000);
  }
  /** RESET ENCODERS (best used when MECHANICAL SLIPPAGE has ocurred) */
  public Command zeroEncoders(){
    armFx1.setSelectedSensorPosition(0);
    armFx2.setSelectedSensorPosition(0);
    return null;
  }

  public void setMotor(double speed){
    armFx1.set(ControlMode.PercentOutput, speed);
  }

  public void setMotorHold(double amps){
    armFx1.set(ControlMode.Current, amps);
  }

  public void setArmPosition(double goal){
    armFx1.set(ControlMode.Position, goal);
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
