// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurretCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TurretSubsystem m_TurretSubsystem;
  private final double m_speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurretCommand(TurretSubsystem subsystem,double speed) {
    m_TurretSubsystem = subsystem;
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
    System.out.println("setting turret speed" + m_speed + " -" + hitLimit());
    if(hitLimit()==false){
      m_TurretSubsystem.turn(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TurretSubsystem.stop();
  }
  
  private boolean hitLimit(){
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
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hitLimit();
  }
}

