// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class zTurretLimelightCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TurretSubsystem m_TurretSubsystem;
  private final Limelight m_limelight;
  private double m_speed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zTurretLimelightCommand(TurretSubsystem subsystem,Limelight limelight, double speed) {
    m_TurretSubsystem = subsystem;
    m_limelight=limelight;
    m_speed=speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_TurretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("auto turet - " + m_limelight.getX());
    if(m_limelight.getX()>Constants.turretOuterLimit){
      m_TurretSubsystem.turn(-m_speed);
    }else if (m_limelight.getX()>Constants.turretInnerLimit){
      m_TurretSubsystem.turn(-m_speed/2);
    }else if (m_limelight.getX()<(0-Constants.turretInnerLimit)){
      m_TurretSubsystem.turn(m_speed/2);
    }else if (m_limelight.getX()<(0-Constants.turretOuterLimit)){
      m_TurretSubsystem.turn(m_speed);
    }else{
      m_TurretSubsystem.stop();
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TurretSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    if (m_TurretSubsystem.hitLeftLimit() && m_TurretSubsystem.isMovingLeft(m_speed)){
      System.out.println("Left Limit");  
      returnValue= true;

    } else if (m_TurretSubsystem.hitRightLimit() && m_TurretSubsystem.isMovingRight(m_speed)){
      System.out.println("Right Limit");  
     
      returnValue= true;
    }
    return returnValue;
  }
}

