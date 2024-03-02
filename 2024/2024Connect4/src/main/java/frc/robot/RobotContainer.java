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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.awt.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.subsystems.*;
import frc.robot.utils.ADABreakBeam;
import frc.robot.utils.Lasercan;
import frc.robot.commands.*;
import frc.robot.Constants.*;
import frc.robot.utils.LED;

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

    private ShooterSubsystem m_Shooter=new ShooterSubsystem(ShooterConstants.shootingMotor1Port,
                                            ShooterConstants.shootingMotor2Port,
                                            ShooterConstants.feederMotorPort);

    private XboxController m_xBoxDriver=new XboxController(Constants.ButtonConstants.xBoxDriverPort);
    private LED m_led= new LED(Constants.LEDConstants.blinkInPWM,Constants.LEDConstants.ledPWM,Constants.LEDConstants.ledCount);
 //   private ADABreakBeam m_adaBreakBeam=new ADABreakBeam(Constants.ADABreakBeamConstants.breakBeamDIO);

    //private Lasercan m_laserCan=new Lasercan(6);

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    
       
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
        final ShooterCommand shooter1Command=new ShooterCommand(m_Shooter,Constants.ShooterConstants.shooterSpeed4);

        JoystickButton shoot1=new JoystickButton(m_xBoxDriver,Constants.ButtonConstants.shooterButton1);
        shoot1.whileTrue(shooter1Command);
        
        final ShooterCommand shooter2Command=new ShooterCommand(m_Shooter,Constants.ShooterConstants.shooterSpeed2);

        JoystickButton shoot2=new JoystickButton(m_xBoxDriver,Constants.ButtonConstants.shooterButton2);
        shoot2.whileTrue(shooter2Command);
        
        final ShooterCommand shooter3Command=new ShooterCommand(m_Shooter,Constants.ShooterConstants.shooterSpeed3);

        JoystickButton shoot3=new JoystickButton(m_xBoxDriver,Constants.ButtonConstants.shooterButton3);
        shoot3.whileTrue(shooter3Command);
        
        final ShooterCommand shooter4Command=new ShooterCommand(m_Shooter,Constants.ShooterConstants.shooterSpeed4);

        JoystickButton shoot4=new JoystickButton(m_xBoxDriver,Constants.ButtonConstants.shooterButton4);
        shoot4.whileTrue(shooter4Command);
    }
       
    public void refreshSmartDashboard(){  
        //System.out.println("ADA Break Beam: "+m_adaBreakBeam.BreakBeam());
        //System.out.println("Laser Can Measurement: "+m_laserCan.GetDistanceInMM());

    }
    
    public void disabledPerioidicUpdates(){
//        m_led.testBlinkIn();
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
          
            // m_led.SetLEDState(Constants.LEDConstants.LEDStates.TELEOP);
            m_led.testBlinkIn();  
        }
    
    public void DisableMode(){
        m_led.SetLEDState(Constants.LEDConstants.LEDStates.OFF);
       
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
  
  public void driveRobot(){
  }
  public void clearAllStickyFaults(){
  Alliance alliance = DriverStation.getAlliance().get();
    //m_robotPDH.clearStickyFaults();
  }
  
}    
