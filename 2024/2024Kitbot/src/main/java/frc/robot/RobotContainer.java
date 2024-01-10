// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.awt.Color;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Constants.*;


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
    private DriveBase m_Drive=new DriveBase(Constants.DriveConstants.leftMotor1Port,Constants.DriveConstants.leftMotor2Port,Constants.DriveConstants.rightMotor1Port, 
                                            Constants.DriveConstants.rightMotor2Port);

    private ShooterSubsystem m_Shooter=new ShooterSubsystem(Constants.ShooterConstants.shootingMotor1Port,Constants.ShooterConstants.shootingMotor2Port);

    private XboxController m_xBoxDriver=new XboxController(Constants.ButtonConstants.xBoxDriverPort);

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        m_Drive.setDefaultCommand(
                new DefaultDriveCommand(m_Drive,
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
        final ShooterCommand shooterCommand=new ShooterCommand(m_Shooter,Constants.ShooterConstants.shooterSpeed);

        JoystickButton shoot=new JoystickButton(m_xBoxDriver,Constants.ButtonConstants.xBoxDriverPort);
        shoot.whileTrue(shooterCommand);
    }
       
    public void refreshSmartDashboard(){  
   
    }
    
    public void disabledPerioidicUpdates(){
        
    }

    public void disableLimelights(){
        
    }
   
    public void resetDriveModes(){
       //m_RobotDrive.resetDriveMode();
    }
   
        public void AutoMode(){
                EnableMode();
                
        }  
        public void TeleopMode(){
                
        }
    
    public void DisableMode(){
       
    }
    public void EnableMode(){
    }

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
        return autoChooser.get();
  }*/

  public void clearAllStickyFaults(){
        //m_robotPDH.clearStickyFaults();
  }
}    
