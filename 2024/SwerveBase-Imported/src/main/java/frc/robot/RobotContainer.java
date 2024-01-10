// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.Autos;


import frc.robot.commands.*;

import frc.robot.subsystems.*;


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
    
    private Swerve swerve = new Swerve();
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

 
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private SendableChooser<Command> m_autoChooser = new SendableChooser<>(); 
    private DigitalInput m_brakeButton = new DigitalInput(Constants.LimitSwitches.brakeButtonPort);
    private boolean m_isBrakeButtonToggled=false;
    private boolean m_brakeButtonPressed=false;
    public RobotContainer() {
      //Named Commands
      NamedCommands.registerCommand("autoscore",new AutoDoNothingCommand()); 
      // Configure the button bindings
        configureButtonBindings();
        //Create Auto Commands
        createAutonomousCommandList();
        swerve.setDefaultCommand(
        swerve.drive(
            () -> m_xBoxDriver.getRawAxis(translationAxis),
            () -> m_xBoxDriver.getRawAxis(strafeAxis),
            () -> m_xBoxDriver.getRawAxis(rotationAxis)));

    }
       
    
    /*m*
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
        
        
}
    
  

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void createAutonomousCommandList(){
          m_autoChooser.setDefaultOption(Autos.autoMode0, new AutoDoNothingCommand());
          m_autoChooser.addOption(Autos.autoMode1, swerve.getPathCommand("StraightLine"));
          m_autoChooser.addOption(Autos.autoMode2, swerve.getPathCommand("StraightLine"));
          SmartDashboard.putData("Auto Chooser",m_autoChooser);
  }
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
  public void checkBrakeButton(){
    if(m_brakeButton.get() && m_brakeButtonPressed==false){ 
      m_brakeButtonPressed=true;
      if(m_isBrakeButtonToggled==false){
                    disableBrakeMode();
                    m_isBrakeButtonToggled=true;
            }else{
                    resetBrakeModetoNormal();
                    m_isBrakeButtonToggled=false;
            }
  }else{

    if (m_brakeButton.get()==false){
            m_brakeButtonPressed=false;
    }
  }
  }
  public void AutonMode(){
  // m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.AUTOMODE);
  // enableLimelights();
    resetBrakeModetoNormal();
    homeRobot();  
  }  
  public void TeleopMode(){
    //m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.TELEOP);
    resetBrakeModetoNormal();
    homeRobot();
    
  }
  private void resetBrakeModetoNormal(){
  }

  private void disableBrakeMode(){
  }
  private void homeRobot(){

  }
}    
