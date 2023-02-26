// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmManual;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.SlowDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_Drivetrain = new Drivetrain();
  private final Intake m_Intake = new Intake();
  private final Arm m_Arm = new Arm();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  //AUTONOMOUS     
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  //DRIVETRAIN DEFAULT--------------------------------------------
   m_Drivetrain.setDefaultCommand(new ArcadeDrive(m_Drivetrain, 
   () -> -m_driverController.getRawAxis(1), 
   () -> m_driverController.getRawAxis(4)));

  //AUTOS
  m_chooser.setDefaultOption("Nothing", null);
  m_chooser.addOption("Drive Distance", new DriveDistance(m_Drivetrain, AutoConstants.autoDistance));


  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //DRIVE MOVEMENT-----------------------------------------------------
    //new JoystickButton(m_driverController, 6).onTrue(new DriveDistance(m_Drivetrain, 1));
    new Trigger(()->{ if(m_driverController.getLeftTriggerAxis() > 0)
    return true;else{return false;}}).whileTrue(new SlowDrive());

    //INTAKE MOVEMENT----------------------------------------------------
    new JoystickButton(m_driverController, Button.kY.value)
    .whileTrue(new IntakeCone(m_Intake, -IntakeConstants.intakePower1));
    new JoystickButton(m_driverController, Button.kX.value)
    .whileTrue(new IntakeCone(m_Intake, IntakeConstants.intakePower2));
    //ARM MOVEMENT-------------------------------------------------------
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
    .whileTrue(new ArmManual(m_Arm, ArmConstants.armPower));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
    .whileTrue(new ArmManual(m_Arm, -ArmConstants.armPower));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected(); 
  }
  
}
