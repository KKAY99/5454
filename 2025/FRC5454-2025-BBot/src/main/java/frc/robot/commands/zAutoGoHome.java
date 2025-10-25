// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutomationConstants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class zAutoGoHome extends Command {
  private IntakeSubsystem m_intake;
  private double m_starttime;
  private double m_duration;
  private double m_speed;
  private double m_targetpos;
  private enum States{
    MOVEHOME, END
  };
  private States m_state;
  public zAutoGoHome(IntakeSubsystem intake,  double speed) {
    m_intake = intake;
    m_speed = speed;

   
    m_state=States.MOVEHOME;
    addRequirements(intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.runIntake(m_speed);
    m_state = States.MOVEHOME;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if(m_intake.getRotatePosition()<=(m_targetpos-AutomationConstants.autoDeadband) 
        && m_intake.getRotatePosition()>=(m_targetpos+AutomationConstants.autoDeadband)){
          m_intake.runIntake(m_speed);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  private boolean moveArm(double targetpos){
    boolean returnValue=false;
    System.out.println("Current Position:" + m_intake.getRotatePositionABS() + " Target Pos:" +targetpos);
    if(m_intake.getRotatePositionABS()<=(targetpos+AutomationConstants.autoABSDeadband) 
    && m_intake.getRotatePositionABS()>=(targetpos-AutomationConstants.autoABSDeadband)){
      returnValue=true;
      
    }else{
      if(m_intake.getRotatePositionABS()>=(targetpos-AutomationConstants.autoABSDeadband)){
        m_intake.Rotate(-AutomationConstants.autoRotateSpeed);
      } else{
        m_intake.Rotate(AutomationConstants.autoRotateSpeed);
      }
    }
    return returnValue;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   boolean endCommand=false;

    switch(m_state){
      case MOVEHOME:        
        if(moveArm(Constants.AutomationConstants.autoHomePosition)){
          m_intake.stopRotate();
          m_intake.stopIntake();
          m_state=States.END;
        }
        break;
     
      default: // will handle END case 
         endCommand=true;
    }

    return endCommand;
  }
}
