// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootRotateSetReferenceCommand extends Command {
  private ShooterSubsystem m_shooter;

  private double m_angle;

  private boolean m_isRunning=false;

  public ShootRotateSetReferenceCommand(ShooterSubsystem shooter,double angle){
    m_shooter=shooter;
    m_angle=angle;
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_isRunning=true;
    Logger.recordOutput("Shooter/ShooterRotateSetReferenceCommand",m_isRunning);
    Logger.recordOutput("Shooter/ShooterRotateSetReferenceAngle",m_angle);

  }

  @Override
  public void end(boolean interrupted){
    m_shooter.stopRotate();
    m_isRunning=false;
    Logger.recordOutput("Shooter/ShooterRotateSetReferenceAngle",0);
    Logger.recordOutput("Shooter/ShooterRotateSetReferenceCommand",m_isRunning);

  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;

    m_shooter.setAngle(m_angle);

    if(m_shooter.getAngle()==m_angle){
      returnValue=true;
    }

    return returnValue;
  }
}
