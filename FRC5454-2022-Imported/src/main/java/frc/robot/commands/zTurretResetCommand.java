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
  private final double m_leftSpeed;
  private final double m_rightSpeed;
  private final double m_targetPos;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zTurretResetCommand(TurretSubsystem subsystem,double leftSpeed,double rightSpeed, double targetPos) {
    m_TurretSubsystem = subsystem;
    m_leftSpeed=leftSpeed;
    m_rightSpeed=rightSpeed;
    m_targetPos=targetPos;

    //Start moving left
    m_speed=leftSpeed;
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
    m_TurretSubsystem.turn(m_speed);
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
    if (m_TurretSubsystem.hitLeftLimit() && m_TurretSubsystem.isMovingLeft()){
      m_TurretSubsystem.setEncoderPosition(0);
      m_speed=m_rightSpeed;
      m_TurretSubsystem.turn(m_speed);
      returnValue= true;
     } else if (m_TurretSubsystem.hitRightLimit() || m_TurretSubsystem.getPosition()>m_targetPos){
      returnValue= true;
  }
     return returnValue;
  }
}

