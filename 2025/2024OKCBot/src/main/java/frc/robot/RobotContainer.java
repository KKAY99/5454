// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ObsidianCanandcolor;
import frc.robot.Constants.*;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeWheelsCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeWheelsSubsystem;
import frc.robot.subsystems.ClimbSubSystem;


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
  private XboxController m_xBoxDriver=new XboxController(Constants.ButtonConstants.xBoxDriverPort);
     private WPI_VictorSPX m_left = new WPI_VictorSPX(1);
     private WPI_VictorSPX m_right = new WPI_VictorSPX(2);
     private DifferentialDrive m_drive=new DifferentialDrive(m_left,m_right);
 
     private ClimbSubSystem m_Climb = new ClimbSubSystem(5);
     private IntakeSubsystem m_intake = new IntakeSubsystem(9);
     private IntakeWheelsSubsystem m_wheels = new IntakeWheelsSubsystem(7);
     /*private m_newleftmotor=VictorSPX()
    //private WPI_VictorSPX m_LeftMotor1=new WPI_VictorSPX(DriveConstants.leftMotor1Port);
    private WPI_VictorSPX m_LeftMotor2=new WPI_VictorSPX(DriveConstants.leftMotor2Port);
    private WPI_VictorSPX m_RightMotor1=new WPI_VictorSPX(DriveConstants.rightMotor1Port);
    private WPI_VictorSPX m_RightMotor2=new WPI_VictorSPX(DriveConstants.rightMotor2Port);
    private MotorControllerGroup m_left = new MotorControllerGroup(m_LeftMotor1,m_LeftMotor2);
    private MotorControllerGroup m_right = new MotorControllerGroup(m_RightMotor1,m_RightMotor2);
    //private SpinMotor m_SpinMotor=new SpinMotor(14);
    private IndexerSubsystem m_Indexer=new IndexerSubsystem(19, 18);
    private IntakeSubsystem m_Intake=new IntakeSubsystem(20, 15);
    private ObsidianCanandcolor m_canColor = new ObsidianCanandcolor(29);*/
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


      JoystickButton intakeUp = new JoystickButton(m_xBoxDriver, 4);
      IntakeCommand armUp = new IntakeCommand(m_intake,0.3);
      intakeUp.whileTrue(armUp);

      JoystickButton intakeDown = new JoystickButton(m_xBoxDriver,1);
      IntakeCommand armDown = new IntakeCommand(m_intake,-0.3);
      intakeDown.whileTrue(armDown);

      JoystickButton intakeIn = new JoystickButton(m_xBoxDriver, 3);
      IntakeWheelsCommand wheelsIn = new IntakeWheelsCommand(m_wheels,0.7);
      intakeIn.whileTrue(wheelsIn);

      JoystickButton intakeOut = new JoystickButton(m_xBoxDriver,2);
      IntakeWheelsCommand wheelsOut = new IntakeWheelsCommand(m_wheels,-0.7);
      intakeOut.whileTrue(wheelsOut);

      JoystickButton climbUp = new JoystickButton(m_xBoxDriver, 5);
      ClimbCommand ClimbUp = new ClimbCommand(m_Climb, 0.7);
      climbUp.whileTrue(ClimbUp);

      JoystickButton climbDown = new JoystickButton(m_xBoxDriver, 6);
      ClimbCommand ClimbDown = new ClimbCommand(m_Climb, -0.7);
      climbDown.whileTrue(ClimbDown);
      
  /*      runMotorButton_1.whileTrue(spinMotorSpeed_1);
      JoystickButton runMotorButton_2 = new JoystickButton(m_xBoxDriver, 2);
      SpinMotorCommand spinMotorSpeed_2 = new SpinMotorCommand(m_SpinMotor, -0.2);
      runMotorButton_2.whileTrue(spinMotorSpeed_2);
      JoystickButton runMotorButton_3 = new JoystickButton(m_xBoxDriver, 3);
      SpinMotorCommand spinMotorSpeed_3 = new SpinMotorCommand(m_SpinMotor, -0.3);
      runMotorButton_3.whileTrue(spinMotorSpeed_3);
      JoystickButton runMotorButton_4 = new JoystickButton(m_xBoxDriver, 4);
      SpinMotorCommand spinMotorSpeed_4 = new SpinMotorCommand(m_SpinMotor, -0.4);ver, 
      runMotorButton_4.whileTrue(spinMotorSpeed_4);
      kButton IntakeButton_1 = new JoystickButton(m_xBoxDriver,1);
      IntakeCommand intakeCommand_1 = new IntakeCommand(m_Intake, 0.95);
      IntakeButton_1.whileTrue(intakeCommand_1);

      JoystickButton IntakeButton_2 = new JoystickButton(m_xBoxDriver, 4);
      IntakeCommand intakeCommand_2 = new IntakeCommand(m_Intake, 0.1);
      IntakeButton_2.whileTrue(intakeCommand_2);

      JoystickButton IndexerButton_1 = new JoystickButton(m_xBoxDriver, 2);
      IndexerCommand indexerCommandSpeed_1 = new IndexerCommand(m_Indexer, 0.5);
      IndexerButton_1.whileTrue(indexerCommandSpeed_1);

      JoystickButton IndexerButton_2 = new JoystickButton(m_xBoxDriver, 3);
      IndexerCommand indexerCommandSpeed_2 = new IndexerCommand(m_Indexer, 0.1);
      IndexerButton_2.whileTrue(indexerCommandSpeed_2);
      */
    }
       
    public void   refreshSmartDashboard(){  
    }
    
    public void disabledPerioidicUpdates(){
        
    }

    public void disableLimelights(){
        
    }
   
    public void driveRobot(){ 
      m_drive.tankDrive(-m_xBoxDriver.getLeftY(),m_xBoxDriver.getRightY());
     // m_drive.arcadeDrive(m_xBoxDriver.getLeftX(),m_xBoxDriver.getRightY());
      m_drive.setDeadband(0.1);
 
    }
   
        public void AutoMode(){
       double startTime = Timer.getFPGATimestamp();
          while(Timer.getFPGATimestamp()<startTime+1){
            System.out.println("Running Auto:" + Timer.getFPGATimestamp() + "  Start time:" + startTime);
            m_left.set(0.3);
            m_right.set(-0.3);
          }
          m_left.set(0);
          m_left.set(0);
                    
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
  
 
  
}    
