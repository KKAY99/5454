// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private ShooterSubsystem m_shooter;

  private double m_speed;
  private double m_baseMotorSpeed;

  private boolean m_isRunning=false;

  public ShootCommand(ShooterSubsystem shooter,double speed,double baseMotorSpeed){
    m_shooter=shooter;
    m_speed=speed;
    m_baseMotorSpeed=baseMotorSpeed;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    //m_shooter.OutPutDistance();
    m_isRunning=true;
    Logger.recordOutput("Shooter/ShooterCommand",m_isRunning);
    Logger.recordOutput("Shooter/ShooterSpeed",m_speed);

  }

  @Override
  public void end(boolean interrupted){
    m_shooter.SlowShootingMotors();  // for testing
    m_isRunning=false;
    m_shooter.ShotTaken();
    Logger.recordOutput("Shooter/ShooterSpeed",0);
    Logger.recordOutput("Shooter/ShooterCommand",m_isRunning);
  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;

    //if(m_shooter.isMotorVelocityAtBase()){
      m_shooter.RunShootingMotors(m_speed);
    //}
    return returnValue;
  }
}
