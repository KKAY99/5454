/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubSystemHigh;
import frc.robot.subsystems.ElevatorSubSystemLow;
import frc.robot.subsystems.DownLiftSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UpLiftSubSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftSpeeds;
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
  private final IntakeSubSystem m_intakeSubSystem =new IntakeSubSystem();
  private final ElevatorSubSystemHigh m_elevatorHigh= new  ElevatorSubSystemHigh();
  private final ElevatorSubSystemLow m_elevatorLow= new ElevatorSubSystemLow();
  private final DownLiftSubSystem m_downLift = new DownLiftSubSystem();
  private final UpLiftSubSystem m_upLift = new UpLiftSubSystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  

  private Joystick m_leftJoystick = new Joystick(InputControllers.kJoystickLeft);
  private Joystick m_rightJoystick = new Joystick(InputControllers.kJoystickRight);
  private XboxController m_xBox = new XboxController(InputControllers.kXboxMain);
  /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    configureButtonBindings();
    m_RobotDrive.setDefaultCommand(new DefaultDrive(m_RobotDrive,()-> -m_leftJoystick.getY(), ()-> -m_rightJoystick.getY()));
    
  } 

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    System.out.println("Button Bindings");
    new JoystickButton(m_leftJoystick, ButtonConstants.elevatorCombinedUp).whenHeld(new ElevatorBothMoveCommand(m_elevatorHigh,0-LiftSpeeds.kLliftUpNormal,m_elevatorLow,LiftSpeeds.kLliftUpNormal));
    new JoystickButton(m_leftJoystick, ButtonConstants.elevatorCombinedDown).whenHeld(new ElevatorBothMoveCommand(m_elevatorHigh,0-LiftSpeeds.kLiftDownMedium,m_elevatorLow,LiftSpeeds.kLiftDownMedium));
    
    new JoystickButton(m_rightJoystick,ButtonConstants.intakeInMain).whenHeld(new IntakeCellMoveCommand(m_intakeSubSystem,IntakeConstants.intakeSpeed));
    new JoystickButton(m_rightJoystick,ButtonConstants.ShooterSpin).whenHeld(new ShooterCommand(m_shooter,ShooterSpeeds.ShooterSpeed10));
    new JoystickButton(m_rightJoystick,ButtonConstants.ShooterCombo10).whenPressed(new ShooterComboCommand(m_elevatorHigh,m_elevatorLow,m_shooter,ShooterSpeeds.ShooterSpeed10));
    new JoystickButton(m_rightJoystick,ButtonConstants.ShooterCombo20).whenPressed(new ShooterComboCommand(m_elevatorHigh,m_elevatorLow,m_shooter,ShooterSpeeds.ShooterSpeed20));
    

    new JoystickButton(m_xBox,ButtonConstants.xboxUpLiftUp).whenHeld(new UpLiftCommand(m_upLift,LiftSpeeds.kLiftDownMedium));
    new JoystickButton(m_xBox,ButtonConstants.xboxUpLiftDown).whenHeld(new UpLiftCommand(m_upLift,LiftSpeeds.kLiftDownSlow));
    new JoystickButton(m_xBox,ButtonConstants.xboxDownLiftUp).whenHeld(new DownLiftCommand(m_downLift, LiftSpeeds.kLliftUpNormal));
    new JoystickButton(m_xBox,ButtonConstants.xboxIntakeIn).whenHeld(new IntakeCellMoveCommand(m_intakeSubSystem,0-IntakeConstants.intakeSpeed));
    new JoystickButton(m_xBox,ButtonConstants.xboxIntakeOut).whenHeld(new IntakeCellMoveCommand(m_intakeSubSystem,IntakeConstants.intakeSpeed));
    
    new JoystickButton(m_xBox,ButtonConstants.xboxComboLift).whenHeld(new LiftComboCommand(m_downLift, m_upLift, 3, .75, 1.2, .30));
    new JoystickButton(m_xBox, ButtonConstants.xboxElevatorCombinedUp).whenHeld(new ElevatorBothMoveCommand(m_elevatorHigh,.8,m_elevatorLow,-0.8));
    new JoystickButton(m_xBox, ButtonConstants.xboxElevatorCombinedDown).whenHeld(new ElevatorBothMoveCommand(m_elevatorHigh,-.6,m_elevatorLow,0.6));
    

    
    //Trigger Check
   
    //15 13
    //    new JoystickButton(m_rightJoystick,2).whenHeld(new DownLiftCommand(m_DownLift,.95));
    //  new JoystickButton(m_rightJoystick,2).whenHeld(new ShooterCommand(.90));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Integer selectedMode) {
    final double autoDriveSpeed=.50;
    final double autoDriveDuration=1.2;
    final double autoDelay=5;
    final double autoShootPower=-.50;
    final double autoNoDelay=0;
    final double autoNoMove=0;
    System.out.println("selectedMode = " + selectedMode);
    Command autoCommand;
    if  (selectedMode==AutoModes.autoShootandMove){
      autoCommand=new AutoShootandMoveCommand(m_elevatorHigh,m_elevatorLow,m_shooter,autoShootPower,m_RobotDrive,autoDriveSpeed,autoDriveDuration,autoNoDelay);
    }
    else if (selectedMode==AutoModes.autoShoot){
      autoCommand=new AutoShootandMoveCommand(m_elevatorHigh,m_elevatorLow,m_shooter,autoShootPower,m_RobotDrive,autoNoMove,autoNoMove,autoNoDelay);
    }
    else if (selectedMode==AutoModes.autoDelayShootandMove){
      autoCommand=new AutoShootandMoveCommand(m_elevatorHigh,m_elevatorLow,m_shooter,autoShootPower,m_RobotDrive,autoDriveSpeed,autoDriveDuration,autoDelay);
    }
    else if (selectedMode==AutoModes.autoDelayMove){
      Timer.delay(autoDelay);  // BAD HACK COODE
      autoCommand=new DriveStraightCommand(m_RobotDrive,autoDriveSpeed,autoDriveDuration);
    }
    else if (selectedMode==AutoModes.autoDelayShoot){
      autoCommand=new AutoShootandMoveCommand(m_elevatorHigh,m_elevatorLow,m_shooter,autoShootPower,m_RobotDrive,autoNoMove,autoNoMove,autoDelay);
    }
    else if (selectedMode==AutoModes.autoNothing){
      autoCommand=new DriveStraightCommand(m_RobotDrive,autoNoMove,autoNoMove);
    }
       else {
      autoCommand=new DriveStraightCommand(m_RobotDrive,autoDriveSpeed,autoDriveDuration); 
    }
       
    return autoCommand;
    //return m_autoCommand;
  }
}
