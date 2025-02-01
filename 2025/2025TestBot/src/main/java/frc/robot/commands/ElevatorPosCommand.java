// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPosCommand extends Command {
  private ElevatorSubsystem m_subSystem;
  private double m_pos;
  private double kDeadband = 5;
  /** Creates a new EndEffectorPosCommand. */
  public ElevatorPosCommand(ElevatorSubsystem subSystem, double pos) {
    m_subSystem = subSystem;
    m_pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subSystem.set_referance(m_pos);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_subSystem.reset_referamce();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    if (m_subSystem.get_motor1pos()>m_pos-kDeadband&&m_subSystem.get_motor1pos()<m_pos+kDeadband){
       returnValue=true;
    }
     
    return returnValue;
  }
}
