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
    private final PnuematicsSubystem m_PnuematicsSubystem = new PnuematicsSubystem(Constants.Pneumatics.HubID,
                                                                   Constants.Pneumatics.moduleType);
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

    static LoggedDashboardNumber frontLeftAngle = new LoggedDashboardNumber("FL Angle", 0);

    static LoggedDashboardNumber frontRightAngle = new LoggedDashboardNumber("FR Angle", 0);

    static LoggedDashboardNumber backLeftAngle = new LoggedDashboardNumber("BL Angle", 0);

    static LoggedDashboardNumber backRightAngle = new LoggedDashboardNumber("BR Angle", 0);
  
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
 
    public RobotContainer() {
        // Configure the button bindings
        createAutoCommands();
        configureButtonBindings();
       /* m_WPIDrive.setDefaultCommand(
                new WPIDriveCommand(m_WPIDrive,
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY(),
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_DriveControlMode.isFieldOrientated()));*/
                

    }

private void createAutoCommands(){
    autoChooser.addDefaultOption(AutoModes.autoMode0, new AutoDoNothingCommand());
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
        m_ledStrip.m_pipeline = m_Limelight.getPipeline();
        m_ledStrip.m_canSeeTarget = m_Limelight.isTargetAvailible();


        m_Limelight.update();
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
                
        }  
        public void TeleopMode(){
                m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.TELEOP);
                resetBrakeModetoNormal();
                homeRobot();
                
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
           
                m_homed=true;
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
        m_PnuematicsSubystem.clearPnuematicStickyFaults();
        m_robotPDH.clearStickyFaults();
  }
}    
