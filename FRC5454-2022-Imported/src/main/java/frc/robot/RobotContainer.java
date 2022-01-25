// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDoNothingCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.InputControllers;
import frc.robot.classes.Limelight;

import com.kauailabs.navx.frc.AHRS;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private AHRS m_ahrs = new AHRS(SPI.Port.kMXP);
  // private final DriveSubsystem m_RobotDrive = new DriveSubsystem(m_ahrs);
  private final SwerveSubsystem m_RobotDrive = new SwerveSubsystem();
  private final Limelight m_Limelight = new Limelight();

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
  // private XboxController m_xBoxOperator = new
  // XboxController(InputControllers.kXboxOperator);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Default Drive
    //TODO: This mapping works for our drive, might not for WPI swerve
    m_RobotDrive.setDefaultCommand(
        new DefaultDriveCommand(m_RobotDrive,
            () -> -m_xBoxDriver.getRightX(),
            () -> m_xBoxDriver.getLeftY(),
            () -> m_xBoxDriver.getLeftX()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Creates a new JoystickButton object for button 1 (xBox A) on m_RobotDrive
    JoystickButton visionButton = new JoystickButton(m_xBoxDriver, 1);
    visionButton.whenHeld(new DefaultDriveCommand(m_RobotDrive,
        () -> m_Limelight.getRotationPower(m_Limelight.getX()),
        () -> m_xBoxDriver.getLeftY(),
        () -> m_xBoxDriver.getLeftX()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Integer selectedMode) {
    Command autoCommand = new AutoDoNothingCommand(); // Default Command is DoNothing
    System.out.println("Autonomouse Selected Mode = " + selectedMode);
    switch (selectedMode) {
      case AutoModes.autoMoveForward:
        // autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
        break;
      case AutoModes.autoMoveBackward:
        // autoCommand= new AutoMoveCommand(m_RobotDrive,.3,2);
        break;
      default:
        autoCommand = new AutoDoNothingCommand();
    }
    return autoCommand;
    // return m_autoCommand;
  }
}
