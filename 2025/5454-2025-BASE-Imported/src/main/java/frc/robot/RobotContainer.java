// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.Constants.InputControllers;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
//@Logged(strategy=Strategy.OPT_IN)
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
 
    private final CommandXboxController m_xBoxDriver = new CommandXboxController(0); 
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);
  
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  
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
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_xBoxDriver.getLeftY() * TunerConstants.kSpeedAt12VoltsMps) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_xBoxDriver.getLeftX() * TunerConstants.kSpeedAt12VoltsMps) // Drive left with negative X (left)
            .withRotationalRate(-m_xBoxDriver.getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
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
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() -> drive.withVelocityX(-m_xBoxDriver.getLeftY() * TunerConstants.kSpeedAt12VoltsMps) // Drive forward with
                                                                                       // negative Y (forward)
        .withVelocityY(-m_xBoxDriver.getLeftX() * TunerConstants.kSpeedAt12VoltsMps) // Drive left with negative X (left)
        .withRotationalRate(-m_xBoxDriver.getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));

  }
  public void TeleopMode(){

  }
}


  
  