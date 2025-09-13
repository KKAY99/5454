// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.CANifier.PinValues;
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
    private ClimbSubsystem m_Climb = new ClimbSubsystem(61);
    private IntakeSubsystem m_Intake =  new IntakeSubsystem(60, 62);
    
    //public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

    // Dashboard inputs
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
  
    private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    //private final SendableChooser<Command> m_autoChooser;
    public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

    private final SendableChooser<Boolean> m_IsDrone = new SendableChooser<>();
    // private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem(Constants.Spindexer.motorPort);
 
 // reversed 9/21

    
    public RobotContainer() {
        //m_autoChooser = AutoBuilder.buildAutoChooser();
        createAutoCommandsList();
        configureNamedCommands();
        configureButtonBindings();
        resetDefaultCommand();
    }


    public void configureNamedCommands() {
        NamedCommands.registerCommand("placeHOLDER",null);
    }

    private void configureButtonBindings(){
        GasPedalCommand gasPedalCommand = new GasPedalCommand(m_swerve, ()->m_xBoxDriver.getRightTriggerAxis());
        m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);

        ClimbCommand runclimbCommand = new ClimbCommand(m_Climb, 0.1);
        m_xBoxDriver.button(1).whileTrue(runclimbCommand);

        IntakeCommand runIntakeCommand = new IntakeCommand(m_Intake, 0.1);
        m_xBoxDriver.button(2).whileTrue(runIntakeCommand);

        IntakeRotateCommand runRotateCommand = new IntakeRotateCommand(m_Intake, 0.1);
        m_xBoxDriver.button(3).whileTrue(runRotateCommand);



        
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

   
    public void AutoMode(){

    }  

    public void TeleopMode(){
    }

    public void DisableMode(){
    }

    public void EnableMode(){
    }


    private void resetDefaultCommand(){
        m_swerve.setDefaultCommand(m_swerve.applyRequestDrive(m_xBoxDriver,translationAxis,strafeAxis,rotationAxis));
      }

  Command returnCommand;
  public Command getAutonomousCommand() {
   Command command = null;//m_autoChooser.getSelected();
   return command;
  }


}    
