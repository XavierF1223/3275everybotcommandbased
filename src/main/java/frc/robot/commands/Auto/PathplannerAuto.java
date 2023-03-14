// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Arm.PIDArm;
import frc.robot.commands.Intake.IntakeCone;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathplannerAuto extends SequentialCommandGroup {
  /** Creates a new PathplannerAuto. */
  Drivetrain m_Drivetrain;
  Intake m_Intake;
  Arm m_Arm;

  public PathplannerAuto(PathPlannerTrajectory auto) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PIDArm(m_Arm, ArmConstants.armSetTopGoal, ArmConstants.armP2),
      Commands.waitSeconds(3),
      new IntakeCone(m_Intake, IntakeConstants.intakePowerCone),
      Commands.waitSeconds(2),
      m_Drivetrain.ramAutoBuilder.fullAuto(m_Drivetrain.auto1),
      Commands.runOnce(()-> m_Drivetrain.followTrajectoryCommand(auto,true))
      .andThen(Commands.runOnce(()->m_Drivetrain.setMotorsVolts(0, 0), m_Drivetrain))
      
    );
  }
}
