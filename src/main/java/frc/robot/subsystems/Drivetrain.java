// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private WPI_TalonFX FL;
  private WPI_TalonFX FR;
  private WPI_TalonFX RL;
  private WPI_TalonFX RR;
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  final int kCountsPerRev = 4096;  //Encoder counts per revolution of the motor shaft.
  final double kGearRatio = 10.71;
  final double kWheelRadiusInches = 3;

  public Drivetrain() {
    FL.setInverted(true);
    FR.setInverted(false);
    RL.follow(FL);
    RR.follow(FR);
    zeroGyroscope();
  }

  public double leftDistance(){
    return (FL.getSelectedSensorPosition() + RL.getSelectedSensorPosition() / 2);
  }

  public double rightDistance(){
    return (FR.getSelectedSensorPosition() + RR.getSelectedSensorPosition() / 2);
  }

  public double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return positionMeters;
  }

  public Command zeroGyroscope() {
    m_navx.zeroYaw();
    m_navx.setAngleAdjustment(DriveConstants.Front_Angle_Offset);
    return null;
  }

  public void setMotors(double left, double right){
    FL.set(left);
    FR.set(-right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Shuffleboard.getTab("Drivetrain")
    .add("Distance Left (M)", nativeUnitsToDistanceMeters(leftDistance()));

    Shuffleboard.getTab("Drivetrain")
    .add("Distance Right (M)", nativeUnitsToDistanceMeters(rightDistance()));
  }
}

