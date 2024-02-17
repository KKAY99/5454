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
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.ButtonConstants;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.TargetHeight;
import frc.robot.Constants.LEDS.Colors;
import frc.robot.classes.DriveControlMode;
import frc.robot.classes.LEDStrip;
import frc.robot.classes.LEDSChargedup;
import frc.robot.classes.Limelight;
import frc.robot.classes.LEDSChargedup.LEDMode;
import frc.robot.commands.*;
import frc.robot.common.drivers.NavX;
import frc.robot.common.drivers.WPISwerveModule;
import frc.robot.common.drivers.NavX.Axis;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.awt.Color;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

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
    private NavX m_NavX = new NavX(SPI.Port.kMXP);
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autonomous Program");
    private final LoggedDashboardChooser<Command> autoDelay = new LoggedDashboardChooser<>("Auto Delay");
   // private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem(Constants.Spindexer.motorPort);
    private final DrivetrainSubsystem m_RobotDrive = new DrivetrainSubsystem(m_NavX); 
    //private final WPIDriveTrainSubsystem m_WPIDrive=new WPIDriveTrainSubsystem(m_NavX);
    private final DriveControlMode m_DriveControlMode = new DriveControlMode();
   
    private final Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight, Constants.LimeLightValues.limelightHeight,
                                                 Constants.LimeLightValues.limelightAngle,Constants.LimeLightValues.kVisionXOffset,80);
   
     private final LEDSChargedup m_ledStrip = new LEDSChargedup(Constants.LEDS.UPPERPORT, Constants.LEDS.UPPERCOUNT);
     private boolean m_disabled=true;
     private boolean m_homed=false;
     private boolean m_isButtonToggled=false;
     private boolean m_buttonPressed=false;
     
     private DigitalInput m_brakeButton = new DigitalInput(Constants.LimitSwitches.brakeButtonPort);

    private final PowerDistribution m_robotPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
   
    static LoggedDashboardString dashDriveMode= new LoggedDashboardString("Drive Mode", "Field"); 
    
    static LoggedDashboardNumber networkTableEntryVisionDistance = new LoggedDashboardNumber("Vision Distance", 0);

  //  static LoggedDashboardNumber networkTableEntryFrontLeftSpeed = new LoggedDashboardNumber("FL Speed", 0);

  //  static LoggedDashboardNumber networkTableEntryFrontRightSpeed = new LoggedDashboardNumber("FR Speed", 0);

  //  static LoggedDashboardNumber networkTableEntryBackLeftSpeed = new LoggedDashboardNumber("BL Speed", 0);

  //  static LoggedDashboardNumber networkTableEntryBackRightSpeed = new LoggedDashboardNumber("BR Speed", 0);

 //   static LoggedDashboardNumber networkTableEntryFrontLeftEncoderActual = new LoggedDashboardNumber("FL Encoder Actual", 0);
 
 //   static LoggedDashboardNumber networkTableEntryFrontRightEncoderActual = new LoggedDashboardNumber("FR Encoder Actual", 0);

 //   static LoggedDashboardNumber networkTableEntryBackLeftEncoderActual = new LoggedDashboardNumber("BL Encoder Actual", 0);

 //   static LoggedDashboardNumber networkTableEntryBackRightEncoderActual = new LoggedDashboardNumber("BR Encoder Actual", 0);

//    static LoggedDashboardNumber networkTableEntryFrontLeftEncoderTarget = new LoggedDashboardNumber("FL Encoder Target", 0);

 //   static LoggedDashboardNumber networkTableEntryFrontRightEncoderTarget = new LoggedDashboardNumber("FR Encoder Target", 0);

  //  static LoggedDashboardNumber networkTableEntryBackLeftEncoderTarget = new LoggedDashboardNumber("BL Encoder Target", 0);

   // static LoggedDashboardNumber networkTableEntryBackRightEncoderTarget = new LoggedDashboardNumber("BR Encoder Target", 0);

    static LoggedDashboardNumber frontLeftAngle = new LoggedDashboardNumber("FL Angle", 0);

    static LoggedDashboardNumber frontRightAngle = new LoggedDashboardNumber("FR Angle", 0);

    static LoggedDashboardNumber backLeftAngle = new LoggedDashboardNumber("BL Angle", 0);

    static LoggedDashboardNumber backRightAngle = new LoggedDashboardNumber("BR Angle", 0);
  
    static LoggedDashboardString ShuffleboardLog = new LoggedDashboardString("ShuffleboardLog", "");

    static LoggedDashboardNumber shuffleboardGyroFused = new LoggedDashboardNumber("Gyro - Fused Heading", 0);

    static LoggedDashboardNumber gryoRoll = new LoggedDashboardNumber("Gyro Roll",0);

    static LoggedDashboardBoolean isOnTarget = new LoggedDashboardBoolean("Is On Target", false);
    
    static LoggedDashboardBoolean isTargetAvailable = new LoggedDashboardBoolean("Is Target Available", false);
    
    static LoggedDashboardBoolean isAtDistanceFromTarget = new LoggedDashboardBoolean("Is At Right Distance", false);
    
    static LoggedDashboardBoolean isAligned = new LoggedDashboardBoolean("Is Aligned With Target", false);

    static LoggedDashboardNumber floorIntakeEncoder = new LoggedDashboardNumber("Intake Rotate Encoder", 0);
    
    static LoggedDashboardNumber elevatorEncoder = new LoggedDashboardNumber("Elevator Encoder", 0);
    
    static LoggedDashboardNumber rotateEncoder = new LoggedDashboardNumber("Rotate Encoder", 0);

    static LoggedDashboardNumber intakeArmsEncoder = new LoggedDashboardNumber("Intake Arms Encoder", 0);
 
    static LoggedDashboardNumber paddleEncoder = new LoggedDashboardNumber("Paddle Encoder", 0);

    static LoggedDashboardBoolean elevatorLimitSwitch = new LoggedDashboardBoolean("Elevator Limit Switch", false);

    static LoggedDashboardBoolean paddleLimitSwitch = new LoggedDashboardBoolean("Paddle Limit Switch", false);

    static LoggedDashboardBoolean intakeArmsLimitSwitch = new LoggedDashboardBoolean("Intake Arms Limit Switch", false);

    static LoggedDashboardBoolean rotateForwardSoftLimit = new LoggedDashboardBoolean("Rotate Forward Soft Limit", false);

    static LoggedDashboardBoolean rotateBackSoftLimit = new LoggedDashboardBoolean("Rotate Backward Soft Limit", false);

    static LoggedDashboardBoolean elevatorMaxLimit = new LoggedDashboardBoolean("Elevator Max Limit", false);

    static LoggedDashboardBoolean rotateHardLimit = new LoggedDashboardBoolean("Rotate Soft Limit", false);

    static LoggedDashboardNumber compressorPressure = new LoggedDashboardNumber("Compressor Pressure", 0);

    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);
 
 // reversed 9/21
    private boolean bClawClose=false;
    private boolean bClawOpen=true;
    private boolean bOpenClawatEnd=true;
    private boolean bNoOpenClawatEnd=false;
    public RobotContainer() {
        // Configure the button bindings
        createAutoCommands();
        configureButtonBindings();
        m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY(),
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_DriveControlMode.isFieldOrientated()));
       /* m_WPIDrive.setDefaultCommand(
                new WPIDriveCommand(m_WPIDrive,
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY(),
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_DriveControlMode.isFieldOrientated()));*/
                

    }

    private void createAutoCommands(){
  
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
      
    }
       
    public void refreshSmartDashboard()
    { 
        gryoRoll.set(m_NavX.getAxis(Axis.ROLL));
      
        m_ledStrip.m_pipeline = m_Limelight.getPipeline();
        m_ledStrip.m_canSeeTarget = m_Limelight.isTargetAvailible();

      
        m_Limelight.update();
        if(true){
                dashDriveMode.set("Field");
        }else{
                dashDriveMode.set("Robot");
        }

        if(m_Limelight.isTargetAvailible()){
                isTargetAvailable.set(true);
        }else{
                isTargetAvailable.set(false);
        }

        if(Math.abs(m_Limelight.getXRaw()) <= Constants.LimeLightValues.kVisionXTolerance){
                isAligned.set(true);
        }else{
                isAligned.set(false);
        }


        if(m_Limelight.getPipeline()==Constants.VisionPipelines.AprilTag){
                m_ledStrip.setPipelineLED();
        }else{
                if(m_Limelight.getPipeline()==Constants.VisionPipelines.BottomTape||m_Limelight.getPipeline()==Constants.VisionPipelines.TopTape){
                        m_ledStrip.setPipelineLED();
                }
        }

        //override disabled led mode
        if(m_disabled){
                m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.DISABLED);
        }
        m_ledStrip.updateLED();
        
}
    
     

     public void disabledPerioidicUpdates(){
        m_ledStrip.updateLED();
       

    }

    public void disableLimelights(){
            m_Limelight.turnLEDOff();
    }
    public void enableLimelights(){
            m_Limelight.turnLEDOn();
            m_disabled=false;
    }
   
    public void resetDriveModes(){
       //m_RobotDrive.resetDriveMode();
    }
   
    
  
        public void AutoMode(){
                EnableMode();
                m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.AUTOMODE);
                enableLimelights();
                resetDriveModes();
                resetBrakeModetoNormal();
                homeRobot();
                //Set Default Pipeline to AprilTags
                m_Limelight.setPipeline(Constants.VisionPipelines.AprilTag);
                
        }  
        public void TeleopMode(){
                m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.TELEOP);
                resetBrakeModetoNormal();
                homeRobot();
                //Set Default Pipeline to AprilTags
                m_Limelight.setPipeline(Constants.VisionPipelines.AprilTag);
                
        }
    private void resetBrakeModetoNormal(){
      
    }

    private void disableBrakeMode(){
     
    }

    public void checkBrakeButton(){
        if(m_brakeButton.get() && m_buttonPressed==false){ 
           m_buttonPressed=true;
           if(m_isButtonToggled==false){
                        disableBrakeMode();
                        m_isButtonToggled=true;
                }else{
                        resetBrakeModetoNormal();
                        m_isButtonToggled=false;
                }
    }else{

         if (m_brakeButton.get()==false){
                m_buttonPressed=false;
         }
    }
}   
    private void homeRobot(){
        
        if(m_homed==false){
          
        } 
    }
    
    public void DisableMode(){
            m_disabled=true;
            m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.DISABLED);
       
    }
    public void EnableMode(){
      m_disabled=false;
}

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        return autoChooser.get();
  }

  public void clearAllStickyFaults(){
    
        m_robotPDH.clearStickyFaults();
  }
}    
