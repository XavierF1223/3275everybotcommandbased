// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DriveConstants;


public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_Drive;
  private final Supplier<Double> xSpeed, zRotation;
  public double MaxDriveOutput;
  private SlewRateLimiter dLimiter = new SlewRateLimiter(DriveConstants.driveSlewLimit);
  //private SlewRateLimiter tLimiter = new SlewRateLimiter(DriveConstants.turnSlewLimit);
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivetrain m_Drive, Supplier<Double> xSpeed, Supplier<Double> zRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Drive = m_Drive;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    addRequirements(m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // grabbing the controller values and squaring them to 'naturalize' inputs
    double drive = xSpeed.get();
    double turn = zRotation.get();
    double drivecalc = Math.copySign(drive * drive, drive);
    double turncalc = Math.copySign(turn * turn, turn);

    // drivespeed maths here instead of in the controller spot
    drivecalc = MathUtil.applyDeadband(drivecalc, DriveConstants.Deadband);
    drivecalc = (drivecalc * DriveConstants.MaxDriveOutput);
    drivecalc = dLimiter.calculate(drivecalc);
    
    // turnspeed maths here instead of in the controller spot
    turncalc = MathUtil.applyDeadband(turncalc, DriveConstants.Deadband);
    turncalc = (turncalc * DriveConstants.MaxTurnOutput);
    //turncalc = tLimiter.calculate(turncalc);

    // finalizing DONT TOUCH
    double left = drivecalc + turncalc;
    double right = drivecalc - turncalc;

    m_Drive.setMotors(left, right);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drive.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
