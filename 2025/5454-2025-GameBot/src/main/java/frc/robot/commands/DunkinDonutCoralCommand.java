// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkinDonutSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DunkinDonutCoralCommand extends Command {
  private DunkinDonutSubsystem m_dunkin;
  private double m_coralSpeed;
  private double m_timeToRun;
  private double m_startTime;
  private boolean m_uselimit;
  private boolean m_useIndexer;
  private double m_indexerHighSpeed;
  private double m_indexerLowSpeed;
  private double m_targetPos;
  private enum States{RUNCORALFORTIME, INDEXERLOW, INDXERHIGH, RUNCORALTOPOS, WAIT, END}
  private States m_currentState = States.INDEXERLOW;
  

  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin, double coralSpeed) {
    m_dunkin = dunkin;
    addRequirements(m_dunkin);
    m_coralSpeed = coralSpeed;
    m_timeToRun=0;
    m_uselimit = false;
    m_useIndexer = false;
    m_indexerHighSpeed = 0;
    m_indexerLowSpeed = 0;
  }

  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin, double coralSpeed,double timeToRun) {
    m_dunkin = dunkin;
    addRequirements(m_dunkin);
    m_coralSpeed = coralSpeed;
    m_timeToRun=timeToRun;
    m_uselimit = false;
    m_useIndexer = false;
    m_indexerHighSpeed = 0;
    m_indexerLowSpeed = 0;
  }

  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin, double coralSpeed,boolean useLimit, boolean useIndexer, double indexerHighSpeed){
    m_dunkin = dunkin;
    addRequirements(m_dunkin);
    m_coralSpeed = coralSpeed;
    m_timeToRun=0;
    m_uselimit = useLimit;
    m_useIndexer = useIndexer;
    m_indexerHighSpeed = indexerHighSpeed;
    m_indexerLowSpeed = 0;

  }

  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin, double coralSpeed,boolean useLimit, boolean useIndexer, double indexerHighSpeed, double indexerLowSpeed){
    m_dunkin = dunkin;
    addRequirements(m_dunkin);
    m_coralSpeed = coralSpeed;
    m_timeToRun=0;
    m_uselimit = useLimit;
    m_useIndexer = useIndexer;
    m_indexerHighSpeed = indexerHighSpeed;
    m_indexerLowSpeed = indexerLowSpeed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    m_startTime=Timer.getFPGATimestamp();
    if(!m_uselimit){
      m_currentState = States.RUNCORALFORTIME;
    }else{
      m_currentState = States.INDEXERLOW;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end");
    m_dunkin.stopCoralMotor();
    m_dunkin.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    switch (m_currentState) {
      case RUNCORALFORTIME:
        m_dunkin.runCoralMotor(m_coralSpeed);
        if(m_timeToRun!=0&&m_timeToRun+m_startTime<Timer.getFPGATimestamp()){
          m_currentState = States.END;
        }
      break;
      case INDEXERLOW:
        m_dunkin.runCoralMotor(m_coralSpeed);
        m_dunkin.runIndexer(m_indexerLowSpeed);
        if(m_dunkin.isCoralAtIndexerLimit()){
          m_currentState = States.INDXERHIGH;
        }
      break;
      case INDXERHIGH:
        m_dunkin.runIndexer(m_indexerHighSpeed);
        if(m_dunkin.isCoralAtBoxLimit()){
          m_dunkin.stopCoralMotor();
          m_dunkin.stopIndexer();
          m_currentState = States.RUNCORALTOPOS;
        }
      break;
      case RUNCORALTOPOS:
        m_targetPos = m_dunkin.getTargetPos(0.9);
        m_dunkin.runCoralMotor(0.05);
        m_currentState = States.WAIT;
      break;
      case WAIT:
        if(m_dunkin.getCoralPos()>m_targetPos){
          m_dunkin.stopCoralMotor();
          m_currentState = States.END;
        }
        break;
      case END:
        returnValue = true;  
    }
    return returnValue;
  }
  
}
