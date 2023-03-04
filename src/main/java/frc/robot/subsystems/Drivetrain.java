// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private TalonFX FL = new TalonFX(DriveConstants.FL);
  private TalonFX FR = new TalonFX(DriveConstants.FR);
  private TalonFX RL = new TalonFX(DriveConstants.RL);
  private TalonFX RR = new TalonFX(DriveConstants.RR);
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
  final double kGearRatio = 10.71;
  final double kWheelRadiusInches = 3;

  public static final double kDefaultMaxOutput = 1.0;

  public Drivetrain() {
    factResetDrive();
    neutralMode(NeutralMode.Brake);
    FL.setInverted(false);
    RL.setInverted(false);
    FR.setInverted(false);
    RR.setInverted(false);
    RL.follow(FL);
    RR.follow(FR);
    zeroGyroscope();
  }

  public void neutralMode(NeutralMode mode){
    FL.setNeutralMode(mode);
    FR.setNeutralMode(mode);
    System.out.println("Drive Motors in " + mode + " Mode!");
}

  public void factResetDrive(){
    FL.configFactoryDefault();
    FR.configFactoryDefault();
    RL.configFactoryDefault();
    RR.configFactoryDefault();
    System.out.println("Drive Motors Reset!");
  }

  public double leftDistance(){
    return (FL.getSelectedSensorPosition() + RL.getSelectedSensorPosition() / 2);
  }

  public double rightDistance(){
    return (FR.getSelectedSensorPosition() + RR.getSelectedSensorPosition() / 2);
  }
  /** Averages BOTH sides of Drivetrain into a distance (meters) */
  public double totalDistance(){
    return ( (Math.abs(encodertoMeter(leftDistance())) + Math.abs(encodertoMeter(rightDistance()))) / 2); //MAKE SURE THIS RETURNS A POSITIVE VALUE ALWAYS
  }

  /**FALCON 500 ENCODER TO METERS*/
  public double encodertoMeter(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return positionMeters;
  }

  public void zeroGyroscope() {
    m_navx.zeroYaw();
    m_navx.setAngleAdjustment(DriveConstants.Front_Angle_Offset);
    System.out.println("Gyroscope Zeroed!");
  }

  public void setMotors(double left, double right){
    FL.set(ControlMode.PercentOutput, left);
    FR.set(ControlMode.PercentOutput, -right);
    SmartDashboard.putNumber("leftout", left);
    SmartDashboard.putNumber("rightout", right);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //SMARTDASHBOARD DEBUG INFO
    SmartDashboard.putNumber("Distance Left", encodertoMeter(leftDistance()));
    SmartDashboard.putNumber("Distance Right", encodertoMeter(rightDistance()));
    SmartDashboard.putNumber("Total Distance", encodertoMeter(totalDistance()));
    SmartDashboard.putNumber("gyro pitch", m_navx.getPitch());
    SmartDashboard.putNumber("gyro yaw", m_navx.getYaw());
    SmartDashboard.putNumber("gyro roll", m_navx.getRoll());
  }
}

