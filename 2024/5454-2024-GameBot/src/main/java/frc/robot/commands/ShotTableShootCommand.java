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
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ShotTable;

public class ShotTableShootCommand extends Command {
  private SendableChooser<Boolean> m_shouldUseDashBoardValuesSendable;

  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  private Limelight m_limeLimelight;

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

  private boolean m_motor1IsAtBase;
  private boolean m_motor2IsAtBase;

  private static int kSlowDownDeadBand=2;

  private enum STATE{SETSHOOTERSPEEDS,RAMPUPSHOOTER,SHOOT,SLOW,END};

  private STATE m_state;

  public ShotTableShootCommand(ShooterSubsystem shooter,IntakeSubsystem intake,Limelight limelight){
    m_shooter=shooter;
    m_intake=intake;

    addRequirements(m_shooter);
  }

 
  @Override
  public void initialize(){
    m_state=STATE.RAMPUPSHOOTER;
  }

  @Override
  public void execute(){
  }

  @Override
  public void end(boolean interrupted){
    m_shooter.SlowShootingMotors();  // for testing
    m_shooter.ShotTaken();
    m_intake.stopIntake();
  }


  @Override
  public boolean isFinished(){
    boolean returnValue=false;


      switch(m_state){
        case SETSHOOTERSPEEDS:
        

        //m_shooter.RunShootingMotors(, kSlowDownDeadBand,true); 
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
                     
          if(m_currentTime+kTimeToRun+5<Timer.getFPGATimestamp()){
              m_motor1TargetSpeed=m_speed1;
              m_motor2TargetSpeed=m_speed2;
              m_state=STATE.SLOW;
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
          returnValue=true;
      }  
    return returnValue;
  }
}
