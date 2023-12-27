// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private DriveSubsystem m_RobotDrive = new DriveSubsystem();
  private ArmSubsystem m_ArmSubsystem = new ArmSubsystem(Constants.Arm.motorPort,Constants.Arm.encoderPort,Constants.Arm.homePos,Constants.Arm.fastSpeed,Constants.Arm.slowSpeed);
  private ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(Constants.ShooterSubsystem.leftShootPort , Constants.ShooterSubsystem .rightShootPort, 
                                                Constants.ShooterSubsystem.snowMotorPort,Constants.ShooterSubsystem.snowMotorSpeed,
                                                Constants.ShooterSubsystem.limitSwitch);
  private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDriver);
  private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Robot Instantiation
    configureBindings();
    m_gyro.calibrate();
    m_gyro.reset();
    
    m_RobotDrive.setDefaultCommand(new DefaultDrive(m_RobotDrive,()->m_xBoxDriver.getLeftX()  , ()-> m_xBoxDriver.getLeftY()));
  }

    
  public void updateDashboard(){
    SmartDashboard.putNumber("Arm Position Encoder",+ m_ArmSubsystem.getEncoderPos());
   // SmartDashboard.putNumber("Max Snow Voltage",m_ShooterSubsystem.getMaxSnowVoltage());
  } 

  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    zMoveArmToPosCommand moveArmPos1=new zMoveArmToPosCommand(m_ArmSubsystem,Constants.Arm.shootPos1);
    zMoveArmToPosCommand moveArmPos2=new zMoveArmToPosCommand(m_ArmSubsystem,Constants.Arm.shootPos2);
    zMoveArmToPosCommand moveArmPos3=new zMoveArmToPosCommand(m_ArmSubsystem,Constants.Arm.shootPos3);

    final MoveArmCommand armDownCommand=new MoveArmCommand(m_ArmSubsystem,-Constants.Arm.manualSpeed, Constants.Arm.minValue,Constants.Arm.maxValue);
    Trigger moveArmDown= new JoystickButton(m_xBoxDriver, Constants.buttonConstants.moveArmDown);
    moveArmDown.whileTrue(armDownCommand);

    final MoveArmCommand armUpCommand=new MoveArmCommand(m_ArmSubsystem,Constants.Arm.manualSpeed, Constants.Arm.minValue,Constants.Arm.maxValue);
    Trigger moveArmUp= new JoystickButton(m_xBoxDriver, Constants.buttonConstants.moveArmUp);
    moveArmUp.whileTrue(armUpCommand);    

    final MoveArmCommand armDown2Command=new MoveArmCommand(m_ArmSubsystem,-Constants.Arm.manualSpeed, Constants.Arm.minValue,Constants.Arm.maxValue);
    final MoveArmCommand armUp2Command=new MoveArmCommand(m_ArmSubsystem,Constants.Arm.manualSpeed, Constants.Arm.minValue,Constants.Arm.maxValue);
    
    Trigger JoystickupUp = new Trigger(() -> (m_xBoxOperator.getLeftY())<-buttonConstants.DeadBand);
    JoystickupUp.whileTrue(armUp2Command);

    Trigger JoystickupDown = new Trigger(() -> (m_xBoxOperator.getLeftY())>buttonConstants.DeadBand);
    JoystickupDown.whileTrue(armDown2Command);



    Trigger buttonArmPos1=new JoystickButton(m_xBoxDriver, Constants.buttonConstants.shootPos1Button);
    Trigger buttonArmPos2=new JoystickButton(m_xBoxDriver, Constants.buttonConstants.shootPos2Button);
    Trigger buttonArmPos3=new JoystickButton(m_xBoxDriver, Constants.buttonConstants.shootPos3Button);
     
    buttonArmPos1.onTrue(moveArmPos1);
    buttonArmPos2.onTrue(moveArmPos2);
    buttonArmPos3.onTrue(moveArmPos3);
     
    final IntakeCubeCommand intakeCubeCommand = new IntakeCubeCommand(m_ShooterSubsystem,Constants.ShooterSubsystem.intakeSpeed);
    Trigger intakeCube = new JoystickButton(m_xBoxOperator, Constants.buttonConstants.intakeCube);
    intakeCube.whileTrue(intakeCubeCommand);

    final ShootCubeCommand shootLowCommand = new ShootCubeCommand(m_ShooterSubsystem, Constants.ShooterSubsystem.shootLowSpeed,
                                             Constants.ShooterSubsystem.delayLowShot,Constants.ShooterSubsystem.shootTime);
    Trigger shootLow=new JoystickButton(m_xBoxOperator, Constants.buttonConstants.shootCubeLow);
    shootLow.toggleOnTrue(shootLowCommand);
    
    final ShootCubeCommand shootMidCommand = new ShootCubeCommand(m_ShooterSubsystem, Constants.ShooterSubsystem.shootMediumSpeed,
                                             Constants.ShooterSubsystem.delayMediumShot,Constants.ShooterSubsystem.shootTime);
    Trigger shootMid=new JoystickButton(m_xBoxOperator, Constants.buttonConstants.shootCubeMedium);
    shootMid.toggleOnTrue(shootMidCommand);
    
    final ShootCubeCommand shootHighCommand = new ShootCubeCommand(m_ShooterSubsystem, Constants.ShooterSubsystem.shootHighSpeed,
                                             Constants.ShooterSubsystem.delayHighShot,Constants.ShooterSubsystem.shootTime);
    Trigger shootHigh=new JoystickButton(m_xBoxOperator, Constants.buttonConstants.shootCubeHigh);
    shootHigh.toggleOnTrue(shootHighCommand);
    
  
  }
  public void AutoMode(){
    //DO LED or anything else we want to indicate AutoMode Engaged
  }
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    // An example command will be run in autonomous
    public Command getAutonomousCommand(Integer selectedMode) {
      Command autoCommand = new AutoDoNothingCommand(); // Default Command is DoNothing
      System.out.println("Autonomouse Selected Mode = " + selectedMode);
      switch (selectedMode) {
        case AutoModes.autoMoveForward:
           autoCommand= new AutoMoveCommand(m_RobotDrive,0,AutoModes.MoveOutDistance);
          break;
          case AutoModes.autoMoveScoreMoveForward:
             autoCommand=new SequentialCommandGroup(new ShootCubeCommand(m_ShooterSubsystem, Constants.ShooterSubsystem.shootMediumSpeed,
                                  Constants.ShooterSubsystem.delayMediumShot,Constants.ShooterSubsystem.shootTime),
                                  new AutoMoveCommand(m_RobotDrive,0,AutoModes.MoveOutDistance));
          break;
        case AutoModes.autoMoveScoreBalance:
          autoCommand=new SequentialCommandGroup(new zMoveArmToPosCommand(m_ArmSubsystem,Constants.Arm.shootPos3),
                                new ShootCubeCommand(m_ShooterSubsystem, Constants.ShooterSubsystem.shootHighSpeed,
                                Constants.ShooterSubsystem.delayHighShot,Constants.ShooterSubsystem.shootTime),
                                new AutoMoveCommand(m_RobotDrive,0,AutoModes.EngageDistance));

                    
        default:
          autoCommand = new AutoDoNothingCommand();
      }
      
      return autoCommand;
      // return m_autoCommand;
  } 
  
  
}
