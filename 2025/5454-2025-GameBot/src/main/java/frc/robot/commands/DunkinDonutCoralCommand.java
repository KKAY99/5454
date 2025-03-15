// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DunkinDonutCoralCommand extends Command {
  private ElevatorSubsystem m_elevator;
  private DunkinDonutSubsystem m_dunkin;
  
  private double m_coralScoreSpeed=0;
  private double m_coralIntakeSpeed=0;
  private double m_timeToRun;
  private double m_startTime;
  private double m_elevatorFPos;

  private boolean m_uselimit;
  private boolean m_useIndexer;
  private boolean m_isRunning;

  private double m_indexerHighSpeed;
  private double m_indexerLowSpeed;
  private double m_targetPos=0;

  private enum States{RUNCORALFORTIME,LOWERELEVATOR,WAITFORELEVATOR,INDEXERLOW,INDEXERHIGH,RUNFORWARD,RUNBACKWARD,END}
  private States m_currentState = States.INDEXERLOW;
  private States m_startState;
  

  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin, double coralIntakeSpeed) {
    m_dunkin = dunkin;
    m_elevator = null;
    addRequirements(m_dunkin);
    m_coralIntakeSpeed = coralIntakeSpeed;
    m_timeToRun=0;
    m_uselimit = false;
    m_useIndexer = false;
    m_indexerHighSpeed = 0;
    m_indexerLowSpeed = 0;
    m_startState = States.INDEXERHIGH;
  }

  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin, double coralScoreSpeed,double timeToRun) {
    m_dunkin = dunkin;
    m_elevator = null;
    addRequirements(m_dunkin);
    m_coralScoreSpeed= coralScoreSpeed;
    m_timeToRun=timeToRun;
    m_uselimit = false;
    m_useIndexer = false;
    m_indexerHighSpeed = 0;
    m_indexerLowSpeed = 0;
    m_startState = States.RUNCORALFORTIME;
  }

  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin, double coralIntakeSpeed,boolean useLimit, boolean useIndexer, double indexerHighSpeed){
    m_dunkin = dunkin;
    m_elevator = null;
    addRequirements(m_dunkin);
    m_coralIntakeSpeed = coralIntakeSpeed;
    m_timeToRun=0;
    m_uselimit = useLimit;
    m_useIndexer = useIndexer;
    m_indexerHighSpeed = indexerHighSpeed;
    m_startState = States.INDEXERHIGH;

  }

  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin,ElevatorSubsystem elevator, double coralIntakeSpeed,boolean useLimit, boolean useIndexer, double indexerHighSpeed){
    m_dunkin = dunkin;
    m_elevator = elevator;
    addRequirements(m_dunkin);
    m_coralIntakeSpeed = coralIntakeSpeed;
    m_timeToRun=0;
    m_uselimit = useLimit;
    m_useIndexer = useIndexer;
    m_indexerHighSpeed = indexerHighSpeed;
    m_startState = States.LOWERELEVATOR;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    m_startTime=Timer.getFPGATimestamp();
    m_currentState = m_startState;
    m_isRunning=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dunkin.stopCoralMotor();
    m_dunkin.stopIndexer();
    m_isRunning=false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    switch (m_currentState) {
      case RUNCORALFORTIME:
        m_dunkin.runCoralMotor(m_coralScoreSpeed);
        if(m_timeToRun!=0&&m_timeToRun+m_startTime<Timer.getFPGATimestamp()){
          m_currentState = States.END;
        }
      break;
      case LOWERELEVATOR:
        if(m_elevator!=null){
          m_elevatorFPos = ElevatorConstants.elevatorLowLimit;
          m_elevator.set_referance(m_elevatorFPos);
          m_currentState = States.WAITFORELEVATOR;
        }else{
          m_currentState = States.INDEXERHIGH;
        }
      break;
      case WAITFORELEVATOR:
        if(m_elevator.CheckCANandColor()){
          m_currentState = States.INDEXERHIGH;
        }
      break;
      case INDEXERLOW:
        m_dunkin.runIndexer(m_indexerLowSpeed);
        if(m_dunkin.isCoralAtIndexerLimit()&&m_coralIntakeSpeed>0){
          m_currentState = States.INDEXERHIGH;
        }
      break;
      case INDEXERHIGH:
        m_dunkin.runCoralMotor(m_coralIntakeSpeed);
        m_dunkin.runIndexer(m_indexerHighSpeed);

        if(m_dunkin.isCoralAtBoxLimit()){
          m_dunkin.stopCoralMotor();
          m_dunkin.stopIndexer();
          m_targetPos=m_dunkin.getCoralPos()+DunkinDonutConstants.clearDoorPosOut;
          m_currentState = States.RUNFORWARD;
        }
      break;
      case RUNFORWARD:
        m_dunkin.runCoralMotor(DunkinDonutConstants.clearDoorSpeedOut);

        if(m_dunkin.getCoralPos()+DunkinDonutConstants.posDeadband>m_targetPos&&
            m_dunkin.getCoralPos()-DunkinDonutConstants.posDeadband<m_targetPos){
            m_dunkin.stopCoralMotor();
            m_targetPos=m_dunkin.getCoralPos()-DunkinDonutConstants.clearDoorPosIn;
            m_currentState=States.RUNBACKWARD;
        }
      break;
      case RUNBACKWARD:
        m_dunkin.runCoralMotor(DunkinDonutConstants.clearDoorSpeedIn);

        if(m_dunkin.getCoralPos()+DunkinDonutConstants.posDeadband>m_targetPos&&
            m_dunkin.getCoralPos()-DunkinDonutConstants.posDeadband<m_targetPos){
            m_dunkin.stopCoralMotor();
            m_currentState=States.END;
        }
      break;
      case END:
        returnValue = true;  
    }

    Logger.recordOutput("Commands/CoralIntake/IsRunning",m_isRunning);
    Logger.recordOutput("Commands/CoralIntake/CurrentState",m_currentState);
    System.out.println("Coral Pos: "+m_dunkin.getCoralPos());
    System.out.println("Target Pos: "+m_targetPos);

    return returnValue;
  }
  
}
