// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class zTurretResetCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TurretSubsystem m_TurretSubsystem;
  private double m_speed;
  private final double m_centerSpeed;
  private final double m_homeSpeed;
  private final double m_targetPos;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zTurretResetCommand(TurretSubsystem subsystem,double homeSpeed,double centerSpeed, double targetPos) {
    m_TurretSubsystem = subsystem;
    m_homeSpeed=homeSpeed;
    m_centerSpeed=centerSpeed;
    m_targetPos=targetPos;

    //Start moving left
    m_speed=homeSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_TurretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Init Turet");
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //fix turret finish function     
   //m_TurretSubsystem.turn(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TurretSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
       return true;
    /* boolean returnValue=false;
    
    System.out.println("Homing");
    if (m_TurretSubsystem.hitRightPhysicalLimit() && (m_speed==m_homeSpeed)) {
      System.out.println("Hit Right Limit");
      m_TurretSubsystem.stop();
      m_TurretSubsystem.setHomeforTurret();
      m_speed=m_centerSpeed;
      m_TurretSubsystem.turn(m_speed);
      
     } else if ((m_speed==m_centerSpeed) && (m_TurretSubsystem.hitLeftLimit() || m_TurretSubsystem.getPosition()<m_targetPos)){
      System.out.println("Hit Left Limit");
      returnValue= true; 
  } 
     return returnValue; */
  }
}

