// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Auto.*;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Intake.*;
import frc.robot.subsystems.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  
  //PATHPLANNER----------------------------------------------------------------------------------
  
  


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_opController =
  new CommandXboxController(OperatorConstants.kOpControllerPort);
  //AUTONOMOUS-------------------------------------------------------------------    
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<String> m_songChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  
  //DRIVETRAIN DEFAULT--------------------------------------------
   m_Drivetrain.setDefaultCommand(new ArcadeDrive(m_Drivetrain, 
   () -> -m_driverController.getRawAxis(1), 
   () -> m_driverController.getRawAxis(4)));

  //AUTOS
  m_chooser.setDefaultOption("Nothing", null);
  m_chooser.addOption("Drive Distance", new DriveDistance(m_Drivetrain, 2.1, -0.15));
  m_chooser.addOption("ConeDrive", new ConeDrive(m_Drivetrain, m_Intake, m_Arm, 2.1));//SHORT SIDE OF TAPE AUTO
  m_chooser.addOption("CubeDrive", new CubeDrive(m_Drivetrain, m_Intake, m_Arm, 2.1));//ON THE SCALE??
  m_chooser.addOption("PPauto", m_Drivetrain.followTrajectoryCommand(m_Drivetrain.auto1, true));
  m_chooser.addOption("ppauto2", new PathplannerAuto(m_Drivetrain.auto1));
  SmartDashboard.putData("Autonomous",m_chooser);
  
  //SONGS
  m_songChooser.setDefaultOption("Nothing", "");
  m_songChooser.addOption("doom.chrp", "doom.chrp");
  //SmartDashboard.putData("Song", m_songChooser);
  //m_Orchestra.LoadMusicSelection(getSong());
  }

  public void neutralMode(NeutralMode mode){
    m_Drivetrain.neutralMode(mode);
  }
  public void resetEncoders(){
    m_Drivetrain.zeroEncoders();
    m_Arm.zeroEncoders();
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
    
    //DRIVE MOVEMENT-------------------------------------------------------------------------------------
    //m_driverController.leftTrigger(0.1).onTrue(new DriveDistance(m_Drivetrain, 1));
    m_driverController.rightTrigger(0.1).whileTrue(new OverDrive());
    //m_driverController.leftTrigger(0.1).and(m_driverController.rightTrigger(0.1)).onTrue(new neutralMode(m_Drivetrain, NeutralMode.Brake));
    m_driverController.leftStick().onTrue(Commands.runOnce(() -> m_Drivetrain.neutralMode(NeutralMode.Brake)));
    //INTAKE---------------------------------------------------------------------------------------------
    m_driverController.y().whileTrue(new IntakeCone(m_Intake, -IntakeConstants.intakePowerCone));
    m_driverController.x().whileTrue(new IntakeCone(m_Intake, IntakeConstants.intakePowerCube));
    m_opController.y().whileTrue(new IntakeCone(m_Intake, -IntakeConstants.intakePowerCone));
    m_opController.x().whileTrue(new IntakeCone(m_Intake, IntakeConstants.intakePowerCube));
    //ARM MOVEMENT MANUAL--------------------------------------------------------------------------------
    m_driverController.start().whileTrue(new ArmManual(m_Arm, -ArmConstants.armPower));
    m_driverController.back().whileTrue(new ArmManual(m_Arm, ArmConstants.armPower));
    //ARM MOVEMENT PID CONTROLLED -----------------------------------------------------------------------
    m_driverController.leftBumper().onTrue(new PIDArm(m_Arm, ArmConstants.armSetStowed, ArmConstants.armP));
    m_driverController.rightBumper().onTrue(new PIDArm(m_Arm, ArmConstants.armSetTopGoal, ArmConstants.armP2));
    m_driverController.a().onTrue(new PIDArm(m_Arm, ArmConstants.armSetMidGoal, ArmConstants.armP));
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

  public String getSong() {
    // An example command will be run in autonomous
    return m_songChooser.toString();
    
  }
  
}
