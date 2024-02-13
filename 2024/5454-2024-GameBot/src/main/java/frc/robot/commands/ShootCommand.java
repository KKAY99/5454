// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.Limelight;

public class ShootCommand extends Command {
  private ShooterSubsystem m_shooter;

  private double m_speed;
  private boolean m_isRunning=false;
  public ShootCommand(ShooterSubsystem shooter,double speed){
    m_shooter=shooter;
    m_speed=speed;

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
    m_shooter.StopShootingMotors();
    m_isRunning=false;
    Logger.recordOutput("Shooter/ShooterSpeed",0);
    Logger.recordOutput("Shooter/ShooterCommand",m_isRunning);

  }

  @Override
  public boolean isFinished(){
    //m_shooter.RunShootingMotors(m_speed);
    return true;
  }
}
