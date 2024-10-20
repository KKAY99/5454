// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private SendableChooser<Boolean> m_shouldUseDashBoardValuesSendable;

  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;

  private NetworkTable m_networkTable;

  private double m_speed1=0;
  private double m_speed2=0;
  private double m_baseMotorSpeed;
  private double m_shooterAngle;
  private double m_currentTime;
  private double kTimeToRun=Constants.ShooterConstants.timeToRunShooter;
  private double m_feederStartTime;
  private double m_targetSpeed=0;
  private double m_motor1TargetSpeed=0;
  private double m_motor2TargetSpeed=0;
  
  private boolean m_motor1IsAtBase=false;
  private boolean m_motor2IsAtBase=false;
  private boolean m_isRunning=false;
  private boolean m_shouldUseDashBoardVals;
  private boolean m_shouldCheckBoardVals;
  private boolean m_setAngle=false;
  private static int kSlowDownDeadBand=2;
  private enum STATE{
   SETANGLE,WAITFORANGLE,RAMPUPSHOOTER,SHOOT,SLOW,END
  }

  private STATE m_state;

  public ShootCommand(ShooterSubsystem shooter,IntakeSubsystem intake,double speed,double baseMotorSpeed){
    m_shooter=shooter;
    m_intake=intake;
    m_speed1=speed;
    m_speed2=speed;
    m_setAngle=false;
    m_baseMotorSpeed=baseMotorSpeed;
    m_shouldUseDashBoardVals=false;

    m_networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");

    GetDashboardShooterVals();
    addRequirements(m_shooter);
  }

  public ShootCommand(ShooterSubsystem shooter,IntakeSubsystem intake,double speed,double baseMotorSpeed,SendableChooser<Boolean> shoulduseDashBoardVals,boolean shouldCheckBoardValues){
    m_shooter=shooter;
    m_intake=intake;
    m_speed1=speed;
    m_speed2=speed;
    m_setAngle=false;
    m_baseMotorSpeed=baseMotorSpeed;
    m_shouldUseDashBoardValuesSendable=shoulduseDashBoardVals;
    m_shouldCheckBoardVals=shouldCheckBoardValues;

    m_networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    addRequirements(m_shooter);
  }
  public ShootCommand(ShooterSubsystem shooter,IntakeSubsystem intake,double speed1,double speed2,double angle,boolean shouldCheckBoardValues){
    m_shooter=shooter;
    m_intake=intake;
    m_speed1=speed1;
    m_speed2=speed2;
    m_shooterAngle=angle;
    m_setAngle=true;
    m_shouldCheckBoardVals=shouldCheckBoardValues;
    m_shouldUseDashBoardValuesSendable=null;
    addRequirements(m_shooter);
  }
 public ShootCommand(ShooterSubsystem shooter,IntakeSubsystem intake,double speed1,double speed2,double angle,boolean setAngle,boolean shouldCheckBoardValues){
    m_shooter=shooter;
    m_intake=intake;
    m_speed1=speed1;
    m_speed2=speed2;
    m_shooterAngle=angle;
    m_setAngle=setAngle;
    m_shouldCheckBoardVals=shouldCheckBoardValues;
    m_shouldUseDashBoardValuesSendable=null;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize(){
    if(m_shouldUseDashBoardValuesSendable!=null){
      m_shouldUseDashBoardVals=m_shouldUseDashBoardValuesSendable.getSelected().booleanValue();
    }else{
      m_shouldUseDashBoardVals=false;
    }

    m_motor1IsAtBase=false;
    m_motor2IsAtBase=false;
    
    m_state=STATE.SETANGLE;
  }

  @Override
  public void execute(){
    GetDashboardShooterVals(); 
    m_isRunning=true;
    Logger.recordOutput("Shooter/ShooterCommand",m_isRunning);
    Logger.recordOutput("Shooter/ShooterSpeed",m_speed1 + " / " + m_speed2);

  }

  @Override
  public void end(boolean interrupted){
    m_shooter.SlowShootingMotors();  // for testing
    m_isRunning=false;
    m_shooter.ShotTaken();
    m_intake.stopIntake();
    Logger.recordOutput("Shooter/ShooterSpeed",0.0);
    Logger.recordOutput("Shooter/ShooterCommand",m_isRunning);
  }

  public void GetDashboardShooterVals(){
    if(m_shouldUseDashBoardVals&&m_shouldCheckBoardVals){
      m_speed1=SmartDashboard.getNumber("Shooter1Veloc",0);
      m_speed2=SmartDashboard.getNumber("Shooter2Veloc",0);
      m_shooterAngle=SmartDashboard.getNumber("ShooterAngle",0);
    }

    /*m_shooterVeloc1=m_networkTable.getEntry("ShooterVeloc1").getNumber(0).doubleValue();
    m_shooterVeloc2=m_networkTable.getEntry("ShooterVeloc2").getNumber(0).doubleValue();
    m_shooterAngle=m_networkTable.getEntry("ShooterAngle").getNumber(0).doubleValue();
    */
  }

  @Override
  public boolean isFinished(){
    if(m_shouldUseDashBoardValuesSendable==null){
    //  System.out.println("Looking up");
    }else{
      System.out.println(m_shouldUseDashBoardValuesSendable.getSelected().booleanValue());
    }
    boolean returnValue=false;
    double angleGap=0;

    if(!m_shouldUseDashBoardVals&&!m_setAngle){
        m_shooter.RunShootingMotors(m_speed1,m_speed2,true);

    if(m_shooter.isMotorVelocitysAtDesiredSpeed(m_speed1,m_speed2)){
      m_shooter.RunFeedRollers(Constants.ShooterConstants.feederSpeed);
      m_intake.runIntake(Constants.IntakeConstants.intakeSpeed);
    } 
    }else{
      switch(m_state){
        case SETANGLE:   
          m_shooter.setAngle(m_shooterAngle);
          m_state=STATE.WAITFORANGLE;
        break;
        case WAITFORANGLE:
          angleGap=Math.abs(m_shooter.getRelativePosition())
                  -Math.abs(m_shooterAngle);
          Logger.recordOutput("Shooter/AngleGap",angleGap);
          if(angleGap<Constants.ShooterConstants.kAngleDeadband && angleGap>-Constants.ShooterConstants.kAngleDeadband ){
            m_shooter.stopRotate();
            m_shooter.RunShootingMotors(m_speed1,m_speed2,false);
            m_state=STATE.RAMPUPSHOOTER;
            m_feederStartTime=Timer.getFPGATimestamp()+Constants.ShooterConstants.kRampUpTime;
            Logger.recordOutput("Shooter/AngleGap",0.00);
          }
        break;
        case RAMPUPSHOOTER:                   
        if(m_shooter.isMotorVelocitysAtDesiredSpeed(m_speed1,m_speed2)){
          m_currentTime=Timer.getFPGATimestamp();
          m_state=STATE.SHOOT;
        }
        break;
        case SHOOT:           
          m_shooter.RunFeedRollers(ShooterConstants.feederSpeed);
          m_intake.runIntake(Constants.IntakeConstants.autoIntakeSpeed);
                     
          if(m_currentTime+kTimeToRun<Timer.getFPGATimestamp()){
              m_motor1TargetSpeed=m_speed1;
              m_motor2TargetSpeed=m_speed2;
              m_state=STATE.END;
          }
        break;
        case SLOW:
        m_shooter.StopFeedRollers();
        m_intake.stopIntake();
        if(m_motor1TargetSpeed<ShooterConstants.baseMotorSpeed-kSlowDownDeadBand&&!m_motor1IsAtBase){
          m_motor1TargetSpeed=m_motor1TargetSpeed+1;
      }else{
          m_motor1IsAtBase=true;
      }

      if(m_motor2TargetSpeed<ShooterConstants.baseMotorSpeed-kSlowDownDeadBand&&!m_motor2IsAtBase){
          m_motor2TargetSpeed=m_motor2TargetSpeed+1;
      }else{
          m_motor2IsAtBase=true;
      }
        
        if(m_motor1IsAtBase&&m_motor2IsAtBase){
            m_state=STATE.END;
        }else{
            m_shooter.RunShootingMotors(m_motor1TargetSpeed,m_motor2TargetSpeed,false);
        }
        break;         
        case END:
          m_shooter.StopFeedRollers();
          m_intake.stopIntake();
          m_shooter.stopShooter();
          returnValue=true;
      }
        }   
    return returnValue;
  }
}
