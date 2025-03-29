// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.TriggerEvent;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonConstants;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.LEDS.Colors;
import frc.robot.classes.DriveControlMode;
import frc.robot.classes.LEDStrip;
import frc.robot.commands.*;
import frc.robot.common.drivers.NavX;
import frc.robot.common.drivers.NavX.Axis;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.classes.BreakBeam;
import frc.robot.classes.DriveControlMode;

import java.awt.Color;

import javax.swing.tree.ExpandVetoException;

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

    //private final SendableChooser<Command> m_autoChooser;

    private final SendableChooser<Boolean> m_IsDrone = new SendableChooser<>();
    // private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem(Constants.Spindexer.motorPort);
    private final DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem(m_NavX); 
    private final DriveControlMode m_DriveControlMode = new DriveControlMode();
    private CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
    private CommandXboxController m_xBoxOperator = new CommandXboxController(InputControllers.kXboxOperator);
    private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);
 
 // reversed 9/21
    private boolean bClawClose=false;
    private boolean bClawOpen=true;
    private boolean bOpenClawatEnd=true;
    private boolean bNoOpenClawatEnd=false;
    
    public RobotContainer() {
        //m_autoChooser = AutoBuilder.buildAutoChooser();
        createAutoCommandsList();
        configureNamedCommands();
        configureButtonBindings();

         m_robotDrive.setDefaultCommand(
                  new DefaultDriveCommand(m_robotDrive,
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY(),
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_DriveControlMode.isFieldOrientated()));    
    }


    public void configureNamedCommands() {
        NamedCommands.registerCommand("placeHOLDER",null);
    }

    private void configureButtonBindings(){
        GasPedalCommand gasPedalCommand = new GasPedalCommand(m_robotDrive, ()->m_xBoxDriver.getRightTriggerAxis());
        m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);
        
    }
    
    private void createAutoCommandsList(){
        /* 
        try{
            SmartDashboard.putData("Auto Chooser", m_autoChooser);
        }catch(Exception e){
            System.out.println("creating autos Failed, Exception" + e.getMessage());
        }*/
    }  

     public void disabledPerioidicUpdates(){
    }

    public void setDriveMode(){
    }

    public void resetDriveModes(){
       m_robotDrive.resetDriveMode();
    }
   
    public void AutoMode(){
    }  

    public void TeleopMode(){
    }

    public void DisableMode(){
    }

    public void EnableMode(){
    }

  Command returnCommand;
  public Command getAutonomousCommand() {
   Command command = null;//m_autoChooser.getSelected();
   return command;
  }


}    
