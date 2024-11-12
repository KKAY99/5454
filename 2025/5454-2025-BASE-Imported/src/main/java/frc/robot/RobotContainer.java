// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.DriverPropertyInfo;
import java.util.List;
import javax.swing.JToggleButton;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;
import frc.robot.Constants.InputControllers;
import frc.robot.commands.*;
import frc.robot.utilities.*;
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
    //STANDARD DRIVING
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    //DRONE DRIVING
    //private final int translationAxis = XboxController.Axis.kRightY.value;
    //private final int strafeAxis = XboxController.Axis.kRightX.value;
    //private final int rotationAxis = XboxController.Axis.kLeftX.value;
    
    private final int rightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    private final int leftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
 
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);
  
    private Swerve m_swerve = new Swerve();
//    private LED m_led=new LED(Constants.LEDConstants.blinkInPWM,Constants.LEDConstants.ledPWM,Constants.LEDConstants.ledCount);
    //private LED m_led= new LED(Constants.LEDConstants.ledPWM,Constants.LEDConstants.ledCount);
    private Limelight m_TurretLimelight = new Limelight(Constants.LimeLightValues.targetHeight,Constants.LimeLightValues.limelightTurretHeight,
                                        Constants.LimeLightValues.limelightTurretAngle);
    /*private Limelight m_StaticLimelight = new Limelight(Constants.LimeLightValues.targetHeight,Constants.LimeLightValues.limelightTurretHeight,
                                        Constants.LimeLightValues.limelightTurretAngle,0,0,Constants.LimeLightValues.staticLimelightName);*/

    private boolean m_isBrakeButtonToggled=false;
    private boolean m_brakeButtonPressed=false;
    private boolean m_HasHomed=false;

    public RobotContainer(){
      //Named Commands
      NamedCommands.registerCommand("autoscore",new AutoDoNothingCommand()); 
      // Configure the button bindings
        configureButtonBindings();
        //Create Auto Commands
        createAutonomousCommandList(); 
        m_swerve.setDefaultCommand(
        m_swerve.drive(
            () -> m_xBoxDriver.getRawAxis(translationAxis),
            () -> m_xBoxDriver.getRawAxis(strafeAxis),
            () -> m_xBoxDriver.getRawAxis(rotationAxis),
            () -> m_xBoxDriver.getRawAxis(rightTriggerAxis)));
        
    }
       
    
    /*m*
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings(){

    }
       
    private void refreshSmartDashboard(){  
      //TODO: ADD BACK WHEN LIMELIGHT ON
      //m_TurretLimelight.LimeLightPeriodic(true);
//      m_swerve.getPose();
        //System.out.println(m_TurretLimelight.getDistance());
      //if(m_turret.IsAtLeftLimit()||m_turret.IsAtRightLimit()){
      //  m_blinkin.SetLEDPrimaryState(LEDStates.ISATLIMIT);
      //}else{
      
    }
    
    

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void createAutonomousCommandList(){
      try{
       
      } catch (Exception e){
          System.out.println("Create Auto Exception: " + e.getMessage());
          }
      }

  public void DisabledInit(){
    
  }
   
  public void DisabledPeriodic(){
    
     
  }
  
  public void AutoPeriodic(){
   
  }

  public void AutonMode(){
   
  }

  public void TeleopPeriodic(){
   
  }

  public Command getAutonomousCommand(){
        return null;
  }

  public void AllPeriodic(){
   
  }

  private void resetDefaultCommand(){
      m_swerve.setDefaultCommand(
        m_swerve.drive(
            () -> m_xBoxDriver.getRawAxis(translationAxis),
            () -> m_xBoxDriver.getRawAxis(strafeAxis),
            () -> m_xBoxDriver.getRawAxis(rotationAxis),
            () -> m_xBoxDriver.getRawAxis(rightTriggerAxis)));

  }
  public void TeleopMode(){

  }
}


  
  