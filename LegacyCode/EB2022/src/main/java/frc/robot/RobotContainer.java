/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Timer;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final DriveSubsystem m_RobotDrive = new DriveSubsystem();
  //private final IntakeLiftSubsystem m_IntakeLiftSubsystem =new IntakeLiftSubsystem();
  //private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  //private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  //private final PneumaticsSubsystem m_Pneumatics = new PneumaticsSubsystem(Constants.Pneumatics.CompressorNode);
  //private final Limelight m_LimeLight = new Limelight(LimeLightValues.targetHeight, LimeLightValues.limelightHeight, Constants.LimeLightValues.limelightAngle);
//  private Joystick m_rightJoystick = new Joystick(InputControllers.kJoystickRight);
  private Joystick m_leftJoystick = new Joystick(InputControllers.kJoystickLeft);
  private Joystick m_rightJoystick = new Joystick(InputControllers.kJoystickRight);
  
 
  
  private XboxController m_xBox = new XboxController(InputControllers.kXboxMain);
  //private XboxController m_xBoxPit = new XboxController(InputControllers.kXboxPit);
  /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    configureButtonBindings();
    m_RobotDrive.setDefaultCommand(new DefaultDrive(m_RobotDrive,()->m_rightJoystick.getY() , ()-> m_leftJoystick.getY()));
    //KK moved to one joystick for arcade
    
   // m_RobotDrive.setDefaultCommand(new DefaultDrive(m_RobotDrive,()->m_leftJoystick.getY() , ()-> m_leftJoystick.getX()));
    
  } 

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    System.out.println("Button Bindings");
    //new POVButton(m_leftJoystick,0).whenHeld(new RobotMoveCommand(m_RobotDrive,-Constants.kSlowMoveLeft,Constants.kSlowMoveRight));    
    //new POVButton(m_leftJoystick,90).whenHeld(new RobotMoveCommand(m_RobotDrive,Constants.kSlowMoveTurn,Constants.kSlowMoveTurn));   
    //new POVButton(m_leftJoystick,180).whenHeld(new RobotMoveCommand(m_RobotDrive,Constants.kSlowMoveLeft,-Constants.kSlowMoveRight));  
    //new POVButton(m_leftJoystick,270).whenHeld(new RobotMoveCommand(m_RobotDrive,-Constants.kSlowMoveTurn,-Constants.kSlowMoveTurn));  
    //new POVButton(m_xBox,90).whenHeld(new ColorWheelSpinCommand (m_ColorWheel,Constants.ColorWheel.RightSpeed));   
    //new POVButton(m_xBox,270).whenHeld(new ColorWheelSpinCommand (m_ColorWheel,Constants.ColorWheel.LeftSpeed));   
    
    //new JoystickButton(m_leftJoystick,ButtonConstants.climberSlow).whenHeld(new ClimberCommand(m_ClimberSubsystem,ClimberSpeeds.ClimberSpeedSlow));
    //new JoystickButton(m_xBox,ButtonConstants.climberSlowXB).whenHeld(new ClimberCommand(m_ClimberSubsystem,ClimberSpeeds.ClimberSpeedSlow));
    //SmartDashboard.putString("Climber Slow","Right-Button " + ButtonConstants.climberSlow);
     //new JoystickButton(m_leftJoystick,ButtonConstants.climberFast).whenHeld(new ClimberCommand(m_ClimberSubsystem,ClimberSpeeds.ClimberSpeedFast));
    //new JoystickButton(m_xBox,ButtonConstants.climberFastXB).whenHeld(new ClimberCommand(m_ClimberSubsystem,ClimberSpeeds.ClimberSpeedFast));
    //SmartDashboard.putString("Climber Fast","Right-Button " + ButtonConstants.climberFast);
    //new JoystickButton(m_rightJoystick,ButtonConstants.climberBackJS).whenHeld(new ClimberCommand(m_ClimberSubsystem,ClimberSpeeds.ClimberSpeedBack));
    //SmartDashboard.putString("Climber Back","Right-Button " + ButtonConstants.climberBackJS);
    //new JoystickButton(m_xBox,ButtonConstants.climberBackXB).whenHeld(new ClimberLimitCommand(m_ClimberSubsystem,ClimberSpeeds.ClimberSpeedBack));
    //SmartDashboard.putString("Climber Back","Xbox-Button " + ButtonConstants.climberBackXB);
    
    //SmartDashboard.putString("Fire Latch","Xbox Button " + ButtonConstants.FlipLatchXB);
    //new JoystickButton(m_xBox,ButtonConstants.FlipLatchXB).whenPressed(new LatchCommand(m_Pneumatics));
    
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Integer selectedMode) {
    Command autoCommand;
    System.out.println("selectedMode = " + selectedMode);
    switch (selectedMode){
      case AutoModes.autoMoveForward:
      autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
      break;
   
    case AutoModes.autoMoveBackward:
      autoCommand= new AutoMoveCommand(m_RobotDrive,.3,2);
      break;
      
    default:
      autoCommand = new AutoDoNothing() ;
    }   
    return autoCommand;
    //return m_autoCommand;
  }
   public double getLimelightDistance(){
     return 0;// m_LimeLight.getDistance();
   } 
}
