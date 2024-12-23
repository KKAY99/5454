// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.utilities.AutoPlanner;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.InputControllers;

//@Logged(strategy=Strategy.OPT_IN)
public class RobotContainer {
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
 
    private final CommandXboxController m_xBoxDriver = new CommandXboxController(0); 
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);

    private SendableChooser<String> m_autoChooser = new SendableChooser<>(); 
  
    private final CommandSwerveDrivetrain m_swerve = TunerConstants.DriveTrain;
  
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

    public RobotContainer(){
      //Named Commands
      NamedCommands.registerCommand("autoscore",new AutoDoNothingCommand()); 
      // Configure the button bindings
        configureButtonBindings();
        //Create Auto Commands
        createAutonomousCommandList(); 
        m_swerve.setDefaultCommand( // m_swerve will execute this command periodically
        m_swerve.applyRequest(() -> drive.withVelocityX(-m_xBoxDriver.getRawAxis(translationAxis) * TunerConstants.kSpeedAt12VoltsMps) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_xBoxDriver.getRawAxis(strafeAxis) * TunerConstants.kSpeedAt12VoltsMps) // Drive left with negative X (left)
            .withRotationalRate(-m_xBoxDriver.getRawAxis(rotationAxis) * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
        
    }
  
    private void configureButtonBindings() {

    }
       
    private void refreshSmartDashboard(){  
    }
    
    

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void createAutonomousCommandList(){
      try{
        m_autoChooser.setDefaultOption(AutoConstants.autoMode1,AutoConstants.autoMode1);
        m_autoChooser.setDefaultOption(AutoConstants.autoMode2,AutoConstants.autoMode2);

        SmartDashboard.putData("Auto Chooser",m_autoChooser);

      } catch (Exception e){
          System.out.println("Create Autos Failed, Exception: " + e.getMessage());
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

  public void TeleopMode(){

  }

  public void TeleopPeriodic(){
   
  }

  public void AllPeriodic(){
   
  }

  public Command getAutonomousCommand(){
    String autoChosen=m_autoChooser.getSelected();
    Command command=getSelectedAuto(autoChosen);
    
    return command;
  }

  public Command getSelectedAuto(String chosenAuto){
    AutoPlanner autoPlan=new AutoPlanner();
    Command command=null;
    switch(chosenAuto){
      case AutoConstants.autoMode1:
      command=new AutoDoNothingCommand();
      break;
      case AutoConstants.autoMode2:
      command=m_swerve.createPathCommand(autoPlan.CreateAutoPath(AutoConstants.AutoPoses.testPose1,AutoConstants.AutoPoses.testPose2));
      break;
    }

    return command;
  }

  private void resetDefaultCommand(){
    m_swerve.setDefaultCommand( // m_swerve will execute this command periodically
    m_swerve.applyRequest(() -> drive.withVelocityX(-m_xBoxDriver.getLeftY() * TunerConstants.kSpeedAt12VoltsMps) // Drive forward with
                                                                                       // negative Y (forward)
        .withVelocityY(-m_xBoxDriver.getLeftX() * TunerConstants.kSpeedAt12VoltsMps) // Drive left with negative X (left)
        .withRotationalRate(-m_xBoxDriver.getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));

  }

}


  
  