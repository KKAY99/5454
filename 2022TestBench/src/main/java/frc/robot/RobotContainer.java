// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ButtonMaps;
import frc.robot.Constants.SmartDashboardLabels;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.MotorCommand;
import frc.robot.commands.TalonMotorCommand;
import frc.robot.commands.MotorCommandJS;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.TalonMotorSubsystem;
import frc.robot.commands.TalonMotorCommandShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Motor Subsystem expects motor port number and if it true/false for brushless motor
  private final MotorSubsystem m_Motor1Subsystem = new MotorSubsystem(Constants.MotorControllers.Motor1,true);
  private final MotorSubsystem m_Motor2Subsystem = new MotorSubsystem(Constants.MotorControllers.Motor2,true);
  private final MotorSubsystem m_Motor3Subsystem = new MotorSubsystem(Constants.MotorControllers.Motor3,true);
  private final MotorSubsystem m_Motor4Subsystem = new MotorSubsystem(Constants.MotorControllers.Motor4,true);
  private final TalonMotorSubsystem m_Shooter1Subsystem = new TalonMotorSubsystem(Constants.MotorControllers.Shooter1,true);
  private final TalonMotorSubsystem m_Shooter2Subsystem = new TalonMotorSubsystem(Constants.MotorControllers.Shooter2,true);
  
   
  private final MotorCommand  m_autoCommand = new MotorCommand(m_Motor1Subsystem,20,"Motor 1");
  private XboxController m_xBoxLeft = new XboxController(Constants.InputControllers.kXboxLeft);
  private XboxController m_xBoxRight= new XboxController(Constants.InputControllers.kXboxLeft);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
  // m_Motor1Subsystem.setDefaultCommand(new MotorCommandJS(m_Motor1Subsystem,()-> m_xBoxLeft.getLeftY(),"V"+SmartDashboardLabels.M1Speed));
   //m_Motor1Subsystem.setDefaultCommand(new MotorCommandJS(m_Motor2Subsystem,()-> m_xBoxLeft.getRightY(),"V"+SmartDashboardLabels.M2Speed));
   //m_Motor1Subsystem.setDefaultCommand(new MotorCommandJS(m_Motor3Subsystem,()-> m_xBoxRight.getLeftY(),"V"+SmartDashboardLabels.M3Speed));
   //m_Motor1Subsystem.setDefaultCommand(new MotorCommandJS(m_Motor4Subsystem,()-> m_xBoxRight.getRightY(),"V"+SmartDashboardLabels.M4Speed));
    createSmartDashboardNumber(SmartDashboardLabels.M1Speed, .75); 
    createSmartDashboardNumber(SmartDashboardLabels.M2Speed, .75); 
    createSmartDashboardNumber(SmartDashboardLabels.M3Speed, .75); 
    createSmartDashboardNumber(SmartDashboardLabels.M4Speed, .75);  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     //  new JoystickButton(m_xBoxLeft,ButtonMaps.SM1ForwardHighSpeed).whenHeld(new MotorCommand(m_Motor1Subsystem,.75,SmartDashboardLabels.M1Speed));  
    // new JoystickButton(m_xBoxLeft,ButtonMaps.SM2ForwardHighSpeed).whenHeld(new MotorCommand(m_Motor2Subsystem,.75,SmartDashboardLabels.M2Speed));
    //   new JoystickButton(m_xBoxLeft,ButtonMaps.SM3ForwardHighSpeed).whenHeld(new MotorCommand(m_Motor3Subsystem,.75,SmartDashboardLabels.M3Speed));
      new JoystickButton(m_xBoxLeft,ButtonMaps.SM4ForwardHighSpeed).whenHeld(new MotorCommand(m_Motor4Subsystem,.75,SmartDashboardLabels.M4Speed));

      // new JoystickButton(m_xBoxRight,ButtonMaps.SM1ReverseHighSpeed).whenHeld(new MotorCommand(m_Motor1Subsystem,-0.75,SmartDashboardLabels.M1Speed));  
      // new JoystickButton(m_xBoxRight,ButtonMaps.SM2ReverseHighSpeed).whenHeld(new MotorCommand(m_Motor2Subsystem,-0.75,SmartDashboardLabels.M2Speed));  
      // new JoystickButton(m_xBoxRight,ButtonMaps.SM3ReverseHighSpeed).whenHeld(new MotorCommand(m_Motor3Subsystem,-0.75,SmartDashboardLabels.M3Speed));  
      // new JoystickButton(m_xBoxRight,ButtonMaps.SM4ReverseHighSpeed).whenHeld(new MotorCommand(m_Motor4Subsystem,-0.75,SmartDashboardLabels.M4Speed));  
       new JoystickButton(m_xBoxLeft,ButtonMaps.SM1ForwardHighSpeed).whenHeld(new TalonMotorCommandShooter(m_Shooter1Subsystem,m_Shooter2Subsystem,0.75));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  public static double createSmartDashboardNumber(String key, double defValue) {

    // See if already on dashboard, and if so, fetch current value
    double value = SmartDashboard.getNumber(key, defValue);
  
    // Make sure value is on dashboard, puts back current value if already set
    // otherwise puts back default value
    SmartDashboard.putNumber(key, value);
  
    return value;
  }   
}
