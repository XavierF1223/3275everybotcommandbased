// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DriveConstants;


public class ArcadeDrive extends CommandBase {
  private final Drivetrain drive;
  private final Supplier<Double> xSpeed, zRotation;
  //private final double xSpeed, zRotation;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivetrain drive, Supplier<Double> xSpeed, Supplier<Double> zRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // grabbing the controller values
    double realTimeSpeed = xSpeed.get();
    double realTimeTurn = zRotation.get();
    //double realTimeSpeed = xSpeed;
    //double realTimeTurn = zRotation;
    // drivespeed maths here instead of in the controller spot
    realTimeSpeed = MathUtil.applyDeadband(realTimeSpeed, DriveConstants.Deadband);
    realTimeSpeed = (realTimeSpeed * DriveConstants.MaxDriveOutput);
    // turnspeed maths here instead of in the controller spot
    realTimeTurn = MathUtil.applyDeadband(realTimeTurn, DriveConstants.Deadband);
    realTimeTurn = (realTimeTurn * DriveConstants.MaxTurnOutput);

    // finalizing
    double left = realTimeSpeed + realTimeTurn;
    double right = realTimeSpeed - realTimeTurn;

    drive.setMotors(left, right);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
