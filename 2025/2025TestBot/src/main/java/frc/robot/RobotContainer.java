// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.awt.Color;
import java.util.concurrent.PriorityBlockingQueue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.utilities.Limelight;

import frc.robot.Constants.*;
import frc.robot.Constants.LedConstants.LEDStates;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.EndEffectorRotateCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.utilities.Leds;
import frc.robot.Constants.NotificationLevel;
import frc.robot.utilities.Elastic;
import frc.robot.subsystems.ServoTest;
import frc.robot.commands.testmoveservo;

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
  Elastic.Notification notification = new Elastic.Notification(Constants.NotificationLevel.QUESTION, "Telop enabled", ".");
  private boolean test = false;
  private final Field2d m_Field2d = new Field2d();
  private XboxController m_xBoxDriver=new XboxController(Constants.ButtonConstants.xBoxDriverPort);
    /* 
    private WPI_VictorSPX m_LeftMotor1=new WPI_VictorSPX(DriveConstants.leftMotor1Port);
    private WPI_VictorSPX m_LeftMotor2=new WPI_VictorSPX(DriveConstants.leftMotor2Port);
    private WPI_VictorSPX m_RightMotor1=new WPI_VictorSPX(DriveConstants.rightMotor1Port);
    private WPI_VictorSPX m_RightMotor2=new WPI_VictorSPX(DriveConstants.rightMotor2Port);
    private MotorControllerGroup m_left = new MotorControllerGroup(m_LeftMotor1,m_LeftMotor2);
    private MotorControllerGroup m_right = new MotorControllerGroup(m_RightMotor1,m_RightMotor2);
    private DifferentialDrive m_drive=new DifferentialDrive(m_left,m_right);*/
    //private SpinMotor m_SpinMotor=new SpinMotor(14);
   // private IndexerSubsystem m_Indexer=new IndexerSubsystem(19, 18);
    //private IntakeSubsystem m_Intake=new IntakeSubsystem(20, 15);
    
    private Leds m_leds = new Leds(Constants.LedConstants.LedCanID, Constants.LedConstants.LedCount);
    private EndEffectorSubsystem m_endEffector = new EndEffectorSubsystem(12,13,10);
    private ElevatorSubsystem m_Elevator = new ElevatorSubsystem(11);
    private ServoTest m_ServoTest = new ServoTest(0);
   // private Limelight m_Limelight=new Limelight();
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
/* 
      JoystickButton runMotorButton_1 = new JoystickButton(m_xBoxDriver, 1);
      SpinMotorCommand spinMotorSpeed_1 = new SpinMotorCommand(m_SpinMotor, -0.1);
      runMotorButton_1.whileTrue(spinMotorSpeed_1);
      JoystickButton runMotorButton_2 = new JoystickButton(m_xBoxDriver, 2);
      SpinMotorCommand spinMotorSpeed_2 = new SpinMotorCommand(m_SpinMotor, -0.2);
      runMotorButton_2.whileTrue(spinMotorSpeed_2);
      JoystickButton runMotorButton_3 = new JoystickButton(m_xBoxDriver, 3);
      SpinMotorCommand spinMotorSpeed_3 = new SpinMotorCommand(m_SpinMotor, -0.3);
      runMotorButton_3.whileTrue(spinMotorSpeed_3);
      JoystickButton runMotorButton_4 = new JoystickButton(m_xBoxDriver, 4);
      SpinMotorCommand spinMotorSpeed_4 = new SpinMotorCommand(m_SpinMotor, -0.4);ver, 
      runMotorButton_4.whileTrue(spinMotorSpeed_4);*/

      /* 
      JoystickButton EndEffectorButton_1 = new JoystickButton(m_xBoxDriver,1);
      EndEffectorCommand effectorCommand_1 = new EndEffectorCommand(m_endEffector,1,.5);
      EndEffectorButton_1.whileTrue(effectorCommand_1);

      
      JoystickButton EndEffectorButton_2 = new JoystickButton(m_xBoxDriver,2);
      EndEffectorCommand effectorCommand_2 = new EndEffectorCommand(m_endEffector,1,-.5);
      EndEffectorButton_2.whileTrue(effectorCommand_2);

      
      JoystickButton EndEffectorButton_3 = new JoystickButton(m_xBoxDriver,3);
      EndEffectorCommand effectorCommand_3 = new EndEffectorCommand(m_endEffector,2,.5);
      EndEffectorButton_3.whileTrue(effectorCommand_3);

      
      JoystickButton EndEffectorButton_4 = new JoystickButton(m_xBoxDriver,1);
      EndEffectorCommand effectorCommand_4 = new EndEffectorCommand(m_endEffector,2,-.5);
      EndEffectorButton_4.whileTrue(effectorCommand_4);*/



      Trigger Right_Stick_Y = new Trigger(() -> Math.abs(m_xBoxDriver.getRightY())>0.1);
      EndEffectorRotateCommand endEffectorRotateCommand = new EndEffectorRotateCommand(m_endEffector, () -> m_xBoxDriver.getRightY());
      Right_Stick_Y.whileTrue(endEffectorRotateCommand);

      JoystickButton Button_X = new JoystickButton(m_xBoxDriver,3);
      EndEffectorRotateCommand endEffectorRotateCommand_2 = new EndEffectorRotateCommand(m_endEffector,() -> 0.1);
      Button_X.whileTrue(endEffectorRotateCommand_2);

      JoystickButton Button_Y = new JoystickButton(m_xBoxDriver,4);
      EndEffectorRotateCommand endEffectorRotateCommand_3 = new EndEffectorRotateCommand(m_endEffector,() -> -0.1);
      Button_Y.whileTrue(endEffectorRotateCommand_3);
     

      Trigger Left_up = new Trigger(() -> Math.abs(m_xBoxDriver.getLeftY())>0.1);
      ElevatorCommand ElevatortCommand= new ElevatorCommand(m_Elevator,() -> m_xBoxDriver.getLeftY());
      Left_up.whileTrue(ElevatortCommand);

            
      JoystickButton Button_A = new JoystickButton(m_xBoxDriver,1);
      testmoveservo servopos = new testmoveservo(m_ServoTest, true);
      //ElevatorCommand ElevatorCommand_4 = new ElevatorCommand(m_Elevator,() -> -0.2);
      Button_A.whileTrue(servopos);//ElevatorCommand_4);

      JoystickButton Button_B = new JoystickButton(m_xBoxDriver,2);
      testmoveservo servoneg = new testmoveservo(m_ServoTest, false);
      //ElevatorCommand ElevatorCommand_3 = new ElevatorCommand(m_Elevator,() -> 0.2);
      Button_B.whileTrue(servoneg);//ElevatorCommand_3);


    }
       
    public void refreshSmartDashboard(){ 
      SmartDashboard.putBoolean("Lined Up", test);
      SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
      SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());
      SmartDashboard.putData("field", m_Field2d);
      SmartDashboard.putData("auto chooser", m_Elevator); //place holder code for auto chooser
      //m_Field2d.getRobotPose() code needed for robot pose
      
     
    }
    
    public void disabledPerioidicUpdates(){
        
    }

    public void disableLimelights(){
        
    }
   
    public void resetDriveModes(){
       //m_RobotDrive.resetDriveMode();
    }
   

    public void AutoMode(){
      Elastic.selectTab("Autonomous");
      EnableMode();
                
    }  
    public void TeleopMode(){
      test = true;
      Elastic.sendNotification(notification);
      Elastic.selectTab("Teleoperated");
      
      m_leds.setLedState(LEDStates.TELEOP);
    }

    public void DisableMode(){
      test = false;
      m_leds.setLedState(LEDStates.DISABLED);
       
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
      
      //m_drive.arcadeDrive(m_xBoxDriver.getLeftX(),m_xBoxDriver.getLeftY());
     // m_drive.setDeadband(0.1);
  }
  public void clearAllStickyFaults(){
  Alliance alliance = DriverStation.getAlliance().get();
    //m_robotPDH.clearStickyFaults();
  }
  
}    
