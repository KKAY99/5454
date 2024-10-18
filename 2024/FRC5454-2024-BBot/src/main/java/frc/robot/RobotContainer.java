// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.FloorIntake;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.TargetHeight;
import frc.robot.Constants.LEDS.Colors;
import frc.robot.classes.DriveControlMode;
import frc.robot.classes.LEDStrip;
import frc.robot.classes.LaserCAN;
import frc.robot.classes.LEDSChargedup;
import frc.robot.classes.Limelight;
import frc.robot.classes.LEDSChargedup.LEDMode;
import frc.robot.commands.*;
import frc.robot.common.drivers.NavX;
import frc.robot.common.drivers.NavX.Axis;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.classes.BreakBeam;

import java.awt.Color;

import javax.swing.tree.ExpandVetoException;

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
    
    private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
    private final LoggedDashboardChooser<Command> autoDelay = new LoggedDashboardChooser<>("Auto Delay");
    private final SendableChooser<Boolean> m_IsDrone = new SendableChooser<>();
    // private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem(Constants.Spindexer.motorPort);
    private final DrivetrainSubsystem m_RobotDrive = new DrivetrainSubsystem(m_NavX); 
    private final IntakeSubsystem m_Intake=new IntakeSubsystem(Constants.FloorIntake.intakeLeftMotorPort, Constants.FloorIntake.intakeRightMotorPort);
    private final ShooterSubsystem m_Shooter = new ShooterSubsystem(Constants.Shooter.shooterMotor1, Constants.Shooter.shooterMotor2, 
                                                                    Constants.Shooter.shooterInclineMotor,Constants.Shooter.shooterFeederMotor);
//    private final ClimbSubsystem m_Climb=new ClimbSubsystem(Constants.Climber.climberPort);
                                                                    //private final WPIDriveTrainSubsystem m_WPIDrive=new WPIDriveTrainSubsystem(m_NavX);
    private final DriveControlMode m_DriveControlMode = new DriveControlMode();
    private final LaserCAN m_LaserCAN = new LaserCAN(Constants.LaserCANConstants.LaserCANID);
    //private final BreakBeam m_BreakBeam=new BreakBeam(Constants.BreakBeam.breakBeamPort,Constants.BreakBeam.breakDistance);
    
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);
 
 // reversed 9/21
    private boolean bClawClose=false;
    private boolean bClawOpen=true;
    private boolean bOpenClawatEnd=true;
    private boolean bNoOpenClawatEnd=false;
    public RobotContainer() {
        // Configure the button bindings
        createAutoCommandsList();
       
        configureButtonBindings();

     /*  m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_xBoxDriver.getRightY(),
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_DriveControlMode.isFieldOrientated()));    
      */ m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY(),
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_DriveControlMode.isFieldOrientated()));
                

    }


    
    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings(){
        IntakeAutoStopCommand intakeIn = new IntakeAutoStopCommand(m_Intake,m_Shooter,m_LaserCAN, Constants.FloorIntake.intakeSpeed);
        JoystickButton intakeInButton = new JoystickButton(m_xBoxDriver, Constants.ButtonConstants.DriverIntakeIn);
        intakeInButton.toggleOnTrue(intakeIn);

        IntakeAutoStopCommand intakeInO = new IntakeAutoStopCommand(m_Intake,m_Shooter,m_LaserCAN, Constants.FloorIntake.intakeSpeed);
        JoystickButton intakeInButtonO = new JoystickButton(m_xBoxOperator, Constants.ButtonConstants.DriverIntakeIn);
        intakeInButtonO.toggleOnTrue(intakeInO);

        /*IntakeCommand intakeIn = new IntakeCommand(m_Intake,m_Shooter, Constants.FloorIntake.intakeSpeed);
        JoystickButton intakeInButton = new JoystickButton(m_xBoxDriver, Constants.ButtonConstants.DriverIntakeIn);
        intakeInButton.whileTrue(intakeIn);*/

        IntakeCommand intakeIn2 = new IntakeCommand(m_Intake,m_Shooter, Constants.FloorIntake.intakeSpeed);
        JoystickButton intakeInButton2 = new JoystickButton(m_xBoxOperator, Constants.ButtonConstants.OperatorIntakeIn);
        intakeInButton2.whileTrue(intakeIn2);

       // IntakeCommand intakeOut = new IntakeCommand(m_Intake,m_Shooter, -Constants.FloorIntake.intakeSpeed);
       // JoystickButton intakeOutButton = new JoystickButton(m_xBoxDriver, Constants.ButtonConstants.DriverIntakeOut);
       // intakeOutButton.whileTrue(intakeOut);

        OuttakeCommand intakeOut2 = new OuttakeCommand(m_Intake,m_Shooter, -Constants.FloorIntake.intakeSpeed);
        JoystickButton intakeOut2Button = new JoystickButton(m_xBoxOperator, Constants.ButtonConstants.OperatorIntakeOut);
        intakeOut2Button.whileTrue(intakeOut2);

       // ClimbCommand climbUp = new ClimbCommand(m_Climb, Constants.Climber.climbUpSpeed);
       // POVButton climbUpPOV = new POVButton(m_xBoxDriver, Constants.ButtonConstants.DriverClimberUpPOV);
       // climbUpPOV.whileTrue(climbUp);

       // ClimbCommand climbDown = new ClimbCommand(m_Climb, Constants.Climber.climbDownSpeed);
       // POVButton climbDownPOV = new POVButton(m_xBoxDriver, Constants.ButtonConstants.DriverClimbDownPOV);
       // climbDownPOV.whileTrue(climbDown);

 
        ShooterInclineCommand InclineUp = new ShooterInclineCommand(m_Shooter, Constants.Shooter.inclineSpeed);
        JoystickButton InclineUpButton = new JoystickButton(m_xBoxOperator, Constants.ButtonConstants.DriverInclineUp);
        InclineUpButton.whileTrue(InclineUp);

        ShooterInclineCommand InclineDown = new ShooterInclineCommand(m_Shooter, -Constants.Shooter.inclineSpeed);
        JoystickButton InclineDownButton = new JoystickButton(m_xBoxOperator, Constants.ButtonConstants.DriverInclineDown);
        InclineDownButton.whileTrue(InclineDown);

        ShooterSetCommand SetInclineHigh = new ShooterSetCommand(m_Shooter, Constants.Shooter.shooterInclinePosHigh);
        JoystickButton SetInclineHighButton = new JoystickButton(m_xBoxDriver, Constants.ButtonConstants.DriverSetInclineHigh);
        SetInclineHighButton.onTrue(SetInclineHigh);
        
        ShooterSetCommand SetInclineLow = new ShooterSetCommand(m_Shooter, Constants.Shooter.shooterInclinePosLow);
        JoystickButton SetInclineLowButton = new JoystickButton(m_xBoxDriver, Constants.ButtonConstants.DriverSetInclineLow);
        SetInclineLowButton.onTrue(SetInclineLow);

        ShooterSetCommand SetInclineMiddle = new ShooterSetCommand(m_Shooter, Constants.Shooter.shooterInclinePosMiddle);
        JoystickButton SetInclineMiddleButton = new JoystickButton(m_xBoxDriver, Constants.ButtonConstants.DriverSetInclineMiddle);
        SetInclineMiddleButton.onTrue(SetInclineMiddle);

        SmartShootManualAimCommand Shoot = new SmartShootManualAimCommand(m_Shooter, Constants.Shooter.ShooterSpeed);
        JoystickButton ShootButton = new JoystickButton(m_xBoxOperator, Constants.ButtonConstants.OperatorShootManual);
        ShootButton.toggleOnTrue(Shoot);

        SmartShootCommand SmartShoot = new SmartShootCommand(m_Shooter, Constants.Shooter.ShooterSpeed,false,true,Constants.Shooter.shooterInclinePosLow);
        JoystickButton SmartShootButton = new JoystickButton(m_xBoxOperator, Constants.ButtonConstants.OperatorShootClose);
        SmartShootButton.toggleOnTrue(SmartShoot);

        SmartShootCommand ShootMid = new SmartShootCommand(m_Shooter, Constants.Shooter.ShooterSpeed,false,true,Constants.Shooter.shooterInclinePosMiddle);
        JoystickButton ShootMidButton = new JoystickButton(m_xBoxOperator, Constants.ButtonConstants.OperatorShootLow);
        ShootMidButton.toggleOnTrue(ShootMid);

        SmartShootCommand ShootHigh = new SmartShootCommand(m_Shooter, Constants.Shooter.ShooterSpeed,false,true,Constants.Shooter.shooterInclinePosHigh);
        JoystickButton ShootHighButton = new JoystickButton(m_xBoxOperator, Constants.ButtonConstants.OperatorShootHigh);
        ShootHighButton.toggleOnTrue(ShootHigh);

        



}
    
    private void createAutoCommandsList(){
    try{
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
        m_autoChooser.setDefaultOption(AutoConstants.autoMode0, AutoConstants.autoMode0);
        m_autoChooser.addOption(AutoConstants.autoMode1, AutoConstants.autoMode1);
        m_autoChooser.addOption(AutoConstants.autoMode2, AutoConstants.autoMode2);
        //m_autoChooser.addOption(AutoConstants.autoMode3, AutoConstants.autoMode3);
        //m_autoChooser.addOption(AutoConstants.autoMode4, AutoConstants.autoMode4);
        //m_autoChooser.addOption(AutoConstants.autoMode5, AutoConstants.autoMode5);
        //m_autoChooser.addOption(AutoConstants.autoMode6, AutoConstants.autoMode6);
         
        SmartDashboard.putData("Is drive inverted", m_IsDrone);
        m_IsDrone.setDefaultOption("Default drive", false);
        m_IsDrone.addOption("Inverted drive", true);

    }catch(Exception e){
        
    }
}  

     public void disabledPerioidicUpdates(){

       

    }

    public void setDriveMode(){
        if(m_IsDrone.getSelected()){
            m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_xBoxDriver.getRightY(),
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_DriveControlMode.isFieldOrientated()));
        }else{
            m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY(),
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_DriveControlMode.isFieldOrientated()));
        }
    }


    public void resetDriveModes(){
       //m_RobotDrive.resetDriveMode();
    }
   
    
  
        public void AutoMode(){

        }  
        public void TeleopMode(){
            //setDriveMode();
            //System.out.println(m_Shooter.getInclineEncoderValue());
            //System.out.println("LaserCan Distance: " + m_LaserCAN.Getdistance());
            System.out.println("Shooter Angle: " + m_Shooter.getInclineEncoderValue());
        
        }






    
    public void DisableMode(){
        m_Shooter.stopShooterIncline();
        System.out.println("LaserCan Distance: " + m_LaserCAN.Getdistance());
        System.out.println("Shooter Angle: " + m_Shooter.getInclineEncoderValue());
    }
    public void EnableMode(){

}

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  Command returnCommand;
  public Command getAutonomousCommand() {
    SmartShootCommand shoot0 = new SmartShootCommand(m_Shooter, Constants.AutoConstants.shoot0Speed, false, false, Constants.Shooter.shooterInclinePosHigh);
    //SmartShootCommand shoot1 = new SmartShootCommand(m_Shooter, Constants.AutoConstants.shoot1Speed);
    ShooterInclineCommand incline0 = new ShooterInclineCommand(m_Shooter, Constants.AutoConstants.inclineSet0);
    ShooterInclineCommand incline1 = new ShooterInclineCommand(m_Shooter, Constants.AutoConstants.inclineSet1);



    String autoChosser = m_autoChooser.getSelected();
        switch (autoChosser) {
            case Constants.AutoConstants.autoMode0:
                returnCommand=new AutoDoNothingCommand();
                break;
            case Constants.AutoConstants.autoMode1:
                returnCommand = new SmartShootManualAimCommand(m_Shooter, Constants.Shooter.ShooterSpeed) ;   
                break;
            case Constants.AutoConstants.autoMode2:
                returnCommand = new SequentialCommandGroup(
                         new SmartShootCommand(m_Shooter, Constants.Shooter.ShooterSpeed,false,true,
                         Constants.Shooter.shooterInclinePosHigh),
                         new AutoMoveTimeCommand(m_RobotDrive,0,2,0.3));
                break;
            default: 
                returnCommand=new AutoDoNothingCommand();
                break;
        } 
        return returnCommand;
  }


}    
