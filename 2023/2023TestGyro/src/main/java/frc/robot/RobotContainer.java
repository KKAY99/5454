// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.testCommand;
import frc.robot.commands.moveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.testCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SpindexCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.Constants.OperatorConstants.InputControllers;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private Joystick m_leftJoystick = new Joystick(InputControllers.kJoystickLeft);
  private Joystick m_rightJoystick = new Joystick(InputControllers.kJoystickRight);
  private ExampleSubsystem m_subsystem = new ExampleSubsystem();
  private DriveSubsystem m_RobotDrive = new DriveSubsystem();
  private IntakeSubsystem m_intake = new IntakeSubsystem();
  private SpindexSubsystem m_SpindexSubsystem = new SpindexSubsystem();
  private testCommand m_command = new testCommand(m_gyro,m_subsystem);
  // private moveCommand m_moveBack = new moveCommand(m_subsystem,-0.3);
  // private moveCommand m_moveForward = new moveCommand(m_subsystem,0.3);
  private IntakeCommand m_intakeInCommand = new IntakeCommand(m_intake,Constants.intake.speedIn);
  private IntakeCommand m_intakeOutCommand = new IntakeCommand(m_intake,Constants.intake.speedOut);
  private SpindexCommand m_SpindexCommand = new SpindexCommand(m_SpindexSubsystem,Constants.spindex.spinspeed);
  
  private XboxController m_xBox = new XboxController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_gyro.reset();
    m_gyro.zeroYaw();
    m_gyro.enableLogging(true);
    while(m_gyro.isCalibrating()){
      System.out.println("Calibrating");
    }
    System.out.println("setting roll zero to " + m_gyro.getRoll());
    m_command.setLevel(m_gyro.getRoll());
    m_RobotDrive.setDefaultCommand(new DefaultDrive(m_RobotDrive,()->m_xBox.getLeftX()  , ()-> m_xBox.getLeftY()));
  }

  public void resetRoll(){
    System.out.println("setting roll zero to " + m_gyro.getRoll());
    m_command.setLevel(m_gyro.getRoll());
  }
  public void updateDashboard(){
    SmartDashboard.putNumber("IMU_Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("IMU_Roll", m_gyro.getRoll());

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    new JoystickButton(m_xBox, 2).whileTrue(m_command);
    //new JoystickButton(m_xBox, 1).whileTrue(m_moveForward);
    //new JoystickButton(m_xBox, 4).whileTrue(m_moveBack);
    new JoystickButton(m_xBox,1).whileTrue(m_intakeInCommand);
    new JoystickButton(m_xBox,4).whileTrue(m_intakeOutCommand);
    new JoystickButton(m_xBox,2).whileTrue(m_SpindexCommand);
    new PIDCommand(null, null, null, null, null);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  
}
