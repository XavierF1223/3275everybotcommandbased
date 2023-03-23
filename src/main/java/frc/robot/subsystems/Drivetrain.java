// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;




public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private TalonFX FL = new TalonFX(DriveConstants.FL);
  private TalonFX FR = new TalonFX(DriveConstants.FR);
  private TalonFX RL = new TalonFX(DriveConstants.RL);
  private TalonFX RR = new TalonFX(DriveConstants.RR);
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private final DifferentialDriveOdometry m_Odometry;
  final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
  final double kGearRatio = 10.71;
  final double kWheelRadiusInches = 3;
  public static final double kTrackwidthMeters = 0;
  public DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  public static final double kDefaultMaxOutput = 1.0;
  public PathPlannerTrajectory auto1;

  public Drivetrain() {
    factResetDrive();
    neutralMode(NeutralMode.Coast);
    FL.setInverted(false);
    RL.setInverted(false);
    FR.setInverted(false);
    RR.setInverted(false);
    FL.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 70, 1));
    RL.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 70, 1));
    FR.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 70, 1));
    RR.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 70, 1));
    RL.follow(FL);
    RR.follow(FR);
    zeroGyroscope();
    m_Odometry = new DifferentialDriveOdometry(m_navx.getRotation2d(), leftDistance(), rightDistance());

    auto1 = PathPlanner.loadPath("auto1", 
    new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(m_navx.getRotation2d(), leftDistance(), rightDistance());
    //SMARTDASHBOARD DEBUG INFO
    SmartDashboard.putNumber("Distance Left", encodertoMeter(leftDistance()));
    SmartDashboard.putNumber("Distance Right", encodertoMeter(rightDistance()));
    SmartDashboard.putNumber("Total Distance", encodertoMeter(totalDistance()));
    SmartDashboard.putNumber("Left Velocity", leftVelocity());
    SmartDashboard.putNumber("RIght Velocity", rightVelocity());
    SmartDashboard.putNumber("gyro pitch", m_navx.getPitch());
    SmartDashboard.putNumber("gyro yaw", m_navx.getYaw());
    SmartDashboard.putNumber("gyro roll", m_navx.getRoll());
    //SmartDashboard.putNumber("driveamps", FL.s)
  }
//-----------------------------------------------------------------------------------------------
/** 
  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
            this.resetOdometry(traj.getInitialPose());
        }
      }),
      new PPRamseteCommand(
          traj, 
          this::getPose2d, // Pose supplier
          new RamseteController(),
          new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
          this.kDriveKinematics, // DifferentialDriveKinematics
          this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
          new PIDController(0.01, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0.01, 0, 0), // Right controller (usually the same values as left controller)
          this::setMotorsVolts, // Voltage biconsumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          this // Requires this drive subsystem
      )
  );
}
*/
//-----------ODOMETRY BASES----------------------------------------------------------------------------------------------------------------------------------------------------
  public Pose2d getPose2d(){
    return m_Odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftVelocity(), rightVelocity());
  }
  public void resetOdometry(Pose2d pose){
    zeroEncoders();
    m_Odometry.resetPosition(m_navx.getRotation2d(), leftDistance(), rightDistance(), pose);
  }
//-------------ENCODER DISTANCE METHODS--------------------------------------------------------------------------------------------------------------------------------------------------
  public double leftDistance(){
    return encodertoMeter(FL.getSelectedSensorPosition() + RL.getSelectedSensorPosition() / 2);}
  public double rightDistance(){
    return encodertoMeter(FR.getSelectedSensorPosition() + RR.getSelectedSensorPosition() / 2);}
  /** Averages BOTH sides of Drivetrain into a distance (meters) */
  public double totalDistance(){
    return (leftDistance() + rightDistance() / 2);} //MAKE SURE THIS RETURNS A POSITIVE VALUE ALWAYS
  public boolean isTol(double distance){
    return (totalDistance() >= distance);
  }
  /**FALCON 500 ENCODER TO METERS*/
  public double encodertoMeter(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return positionMeters;
  }
//-------------ENCODER VELOCITY METHODS--------------------------------------------------------------------------------------------------------------------------------------------------
    public double leftVelocity(){
      return encoderVelocitytoMs(FL.getSelectedSensorVelocity() + RL.getSelectedSensorVelocity() / 2);
    }
    public double rightVelocity(){
      return encoderVelocitytoMs(FR.getSelectedSensorVelocity() + RR.getSelectedSensorVelocity() / 2);
    }
    /**FALCON 500 ENCODER VELOCITY TO METERS PER SECOND*/
    public double encoderVelocitytoMs(double velocity){
      double timeCon = velocity * 10;
      double motorRotationsPerSec = timeCon / kCountsPerRev;
      double wheelRotationsPerSec = motorRotationsPerSec / kGearRatio;
      double postionMetersPerSec = wheelRotationsPerSec * (2*Math.PI*Units.inchesToMeters(kWheelRadiusInches));
      return postionMetersPerSec;
    }
//------------------GYRO METHODS---------------------------------------------------------------------------------------------------------------------------------------------
  public void zeroGyroscope() {
    m_navx.zeroYaw();
    m_navx.setAngleAdjustment(DriveConstants.Front_Angle_Offset);
    System.out.println("Gyroscope Zeroed!");
  }

  public double getHeading(){return m_navx.getRotation2d().getDegrees();}
  public double getTurnRate(){return -m_navx.getRate();}
//----------------MISC METHODS-----------------------------------------------------------------------------------------------------------------------------------------------
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

  public void zeroEncoders(){
    FL.setSelectedSensorPosition(0);
    FR.setSelectedSensorPosition(0);
    RL.setSelectedSensorPosition(0);
    RR.setSelectedSensorPosition(0);
  }

  public void setMotors(double left, double right){
    FL.set(ControlMode.PercentOutput, left);
    FR.set(ControlMode.PercentOutput, -right);
  }

  public void setMotorsVolts(double left, double right){ //wtf??
    left = left / RobotController.getBatteryVoltage();
    right = right / RobotController.getBatteryVoltage();
    FL.set(ControlMode.PercentOutput, left);
    FR.set(ControlMode.PercentOutput, -right);
  }

}
