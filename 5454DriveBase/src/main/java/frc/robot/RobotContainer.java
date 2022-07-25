// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ADXRS450_Gyro m_gyro=new ADXRS450_Gyro();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem(m_gyro);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem, 0, 0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final ExampleCommand TestCommandForward= new ExampleCommand(m_exampleSubsystem, 0.5, -0.5);
    final ExampleCommand TestCommandBackward= new ExampleCommand(m_exampleSubsystem, -0.5, 0.5);
    final ExampleCommand TestCommandLeft= new ExampleCommand(m_exampleSubsystem, 0.5, 0.2);
    final ExampleCommand TestCommandRight= new ExampleCommand(m_exampleSubsystem, -0.2, -0.5);
    final XboxController m_driver = new XboxController(0);
    final JoystickButton testbuttonF = new JoystickButton(m_driver, 4);
    final JoystickButton testbuttonB = new JoystickButton(m_driver, 1);
    final JoystickButton testbuttonL = new JoystickButton(m_driver, 3);
    final JoystickButton testbuttonR = new JoystickButton(m_driver, 2);
    testbuttonF.whenHeld(TestCommandForward);
    testbuttonB.whenHeld(TestCommandBackward);
    testbuttonL.whenHeld(TestCommandLeft);
    testbuttonR.whenHeld(TestCommandRight);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
   }
public void updategyro() {
  System.out.println(m_gyro.getAngle());
}
private final DriveSubsystem m_RobotDrive = new DriveSubsystem();
  
 public RobotContainer() {
    // Configure the button bindings

    configureButtonBindings();
    //m_RobotDrive.setDefaultCommand(new DefaultDrive(m_RobotDrive,()->m_rightJoystick.getY() , ()-> m_leftJoystick.getY()));
    //KK moved to one joystick for arcade
    
    m_RobotDrive.setDefaultCommand(new DefaultDrive(m_RobotDrive,()->m_leftJoystick.getY() , ()-> m_leftJoystick.getX()));
    
  } 

}
