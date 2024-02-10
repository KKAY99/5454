// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.Limelight;

public class ShootCommand extends Command {
  private ShooterSubsystem m_shooter;

  private double m_speed;

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
  }

  @Override
  public void end(boolean interrupted){
    //m_shooter.StopShootingMotors();
  }

  @Override
  public boolean isFinished(){
    //m_shooter.RunShootingMotors(m_speed);
    return true;
  }
}
