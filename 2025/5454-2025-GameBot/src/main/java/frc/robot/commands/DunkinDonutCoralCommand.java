// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkinDonutSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DunkinDonutCoralCommand extends Command {
  private DunkinDonutSubsystem m_dunkin;
  private double m_speed;
  private double m_timeToRun;
  private double m_startTime;


  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin, double speed) {
    m_dunkin = dunkin;
    addRequirements(m_dunkin);
    m_speed = speed;
    m_timeToRun=0;
  }

  public DunkinDonutCoralCommand(DunkinDonutSubsystem dunkin, double speed,double timeToRun) {
    m_dunkin = dunkin;
    m_speed = speed;
    m_timeToRun=timeToRun;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    m_startTime=Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_dunkin.runCoralMotor(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dunkin.stopCoralMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    if(m_timeToRun!=0&&m_timeToRun+m_startTime<Timer.getFPGATimestamp()){
      returnValue=true;
    }
    return returnValue;
  }
}
