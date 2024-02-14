// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.Limelight;

public class ShootRotateCommand extends Command {
  private ShooterSubsystem m_shooter;

  private double m_speed;

  private boolean m_isRunning=false;

  public ShootRotateCommand(ShooterSubsystem shooter,double speed){
    m_shooter=shooter;
    m_speed=speed;
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_isRunning=true;
    Logger.recordOutput("Shooter/ShooterRotateCommand",m_isRunning);
    Logger.recordOutput("Shooter/ShooterRotateSpeed",m_speed);

  }

  @Override
  public void end(boolean interrupted){
    m_shooter.stopRotate();
    m_isRunning=false;
    Logger.recordOutput("Shooter/ShooterRotateSpeed",0);
    Logger.recordOutput("Shooter/ShooterRotateCommand",m_isRunning);

  }

  @Override
  public boolean isFinished(){
    m_shooter.RotateShooter(m_speed);
    return false;
  }
}
