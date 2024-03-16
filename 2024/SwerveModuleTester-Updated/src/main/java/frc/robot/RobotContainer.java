// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.InputControllers;



import frc.robot.commands.*;

import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * 
 * 
 * 












 
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    
    private int drivePort=31;
    private int turnPort=41;
    private int encoderPort=51;
    // Front Right Encoder = 51 , front Left Encoder = 52
    private final TestSwerveModuleSubsystem m_RobotDrive = new TestSwerveModuleSubsystem(drivePort,turnPort,encoderPort);

    // CAN BUS tESTING - NOT PART OF REGULAR CODE
    private CANSparkMax x1 =new CANSparkMax(32, MotorType.kBrushless);
    private CANSparkMax x2 =new CANSparkMax(33, MotorType.kBrushless);
    private CANSparkMax x3 =new CANSparkMax(34, MotorType.kBrushless);
    private CANSparkMax x4 =new CANSparkMax(42, MotorType.kBrushless);
    private CANSparkMax x5 =new CANSparkMax(43, MotorType.kBrushless);
    private CANSparkMax x6 =new CANSparkMax(44, MotorType.kBrushless);
    
    private CANSparkMax x7 =new CANSparkMax(18, MotorType.kBrushless);
    private CANSparkMax x8 =new CANSparkMax(19, MotorType.kBrushless);
    private CANSparkMax x9 =new CANSparkMax(20, MotorType.kBrushless);
    private CANSparkMax x10 =new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax x11 =new CANSparkMax(23, MotorType.kBrushless);
    private CANSparkMax x12 =new CANSparkMax(21, MotorType.kBrushless);
    
 
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
 
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY()));
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
        TurnReferenceCommand pos1command = new TurnReferenceCommand(m_RobotDrive, 100);
        TurnReferenceCommand pos2command = new TurnReferenceCommand(m_RobotDrive, 220);
        Trigger movePos1 = new JoystickButton(m_xBoxDriver, 1);
        Trigger movePos2 = new JoystickButton(m_xBoxDriver, 2);
        movePos1.whileTrue(pos1command);
        movePos2.whileTrue(pos2command);
        SmartDashboard.putData("Turn Turn Motor Right", new TurnCommand(m_RobotDrive,0.25));
        SmartDashboard.putData("Turn Turn Motor Left", new TurnCommand(m_RobotDrive,-0.25));
        SmartDashboard.putData("Turn Drive Motor Forward", new DriveCommand(m_RobotDrive,0.25));
        SmartDashboard.putData("Turn Drive Motor Back", new DriveCommand(m_RobotDrive,-0.25));

//        public abstract void setReference(double setpoint, double feedforward);
  

        
    }
       
    public void refreshSmartDashboard()
    {  
        SmartDashboard.putNumber("Module Speed", m_RobotDrive.getDriveSpeed());
        SmartDashboard.putNumber("Turn Speed", m_RobotDrive.getTurnSpeed());
        SmartDashboard.putNumber("Can Coder Relative Position",m_RobotDrive.getTurnEncoderPosition());
        SmartDashboard.putNumber("Can Coder Absolute Position",m_RobotDrive.getTurnEncoderAbsolutePosition());
        SmartDashboard.putNumber("Turn Relative Encoder Position",m_RobotDrive.getTurnRelativeEncoderPosition());
        
   
        
}
    
  

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        return null;
  }


}    
