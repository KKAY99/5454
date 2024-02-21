// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InputControllers;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class FeedRollerCommand extends Command {
  private ShooterSubsystem m_shooter;

  private double m_speed;
  private double m_baseMotorSpeed;
  private XboxController m_controller;

  private boolean m_isRunning=false;

  public FeedRollerCommand(ShooterSubsystem shooter,double speed,XboxController controller){
    m_shooter=shooter;
    m_speed=speed;
    m_controller=controller;
    //m_baseMotorSpeed=baseMotorSpeed;
  }

  @Override
  public void initialize(){
      m_controller.setRumble(RumbleType.kBothRumble,InputControllers.kRumbleMedium);
    }

  @Override
  public void execute(){
    m_isRunning=true;
    m_shooter.RunFeedRollers(m_speed);
    
    Logger.recordOutput("Shooter/FeedRollerCommand",m_isRunning);
    Logger.recordOutput("Shooter/FeedRollerSpeed",m_speed);

  }

  @Override
  public void end(boolean interrupted){
    m_shooter.StopFeedRollers();
      m_controller.setRumble(RumbleType.kBothRumble,InputControllers.kRumbleoff);
    
    m_isRunning=false;
    Logger.recordOutput("Shooter/FeedRollerSpeed",0);
    Logger.recordOutput("Shooter/FeedRollerCommand",m_isRunning);
  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;
     return returnValue;
  }
}
