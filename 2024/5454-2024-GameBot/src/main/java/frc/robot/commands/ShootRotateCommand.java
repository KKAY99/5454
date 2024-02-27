// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootRotateCommand extends Command {
  private ShooterSubsystem m_shooter;

  private double m_speed=0;

  private boolean m_isRunning=false;
  private boolean m_isSupplierMode=false;
  private DoubleSupplier m_doubleSupplier=null;
  private boolean m_hasHitLimit=false;
  public ShootRotateCommand(ShooterSubsystem shooter,double speed){
    m_shooter=shooter;
    m_speed=speed;
    m_isSupplierMode=true;
  }
  public ShootRotateCommand(ShooterSubsystem shooter,DoubleSupplier speedSupplier){
    m_shooter=shooter;
    m_speed=0;
    m_doubleSupplier=speedSupplier;
    m_isSupplierMode=true;
  }
  @Override
  public void initialize(){
    m_hasHitLimit=false;
  }

  @Override
  public void execute(){
    m_isRunning=true;
    //if running supplier mode get supplier (joystick axis for speed)
    if (m_isSupplierMode) {
      m_speed=m_doubleSupplier.getAsDouble();
    }
    if(m_shooter.hasHitRotateLimit(m_speed)){
      m_speed=0;  //stop speed if at limit
      m_hasHitLimit=true;
    }
    m_shooter.RotateShooter(m_speed);
    Logger.recordOutput("Shooter/ShooterRotateCommand",m_isRunning);
    Logger.recordOutput("Shooter/ShooterRotateSpeed",m_speed);
    Logger.recordOutput("Shooter/ShooterHitSoftLimit",m_hasHitLimit);

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
    return m_hasHitLimit;
  }
}
