// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WinchSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WinchCommand extends Command {
  WinchSubsystem m_winch;
  double m_speed;

  public WinchCommand(WinchSubsystem winch, double speed) {
    m_winch = winch;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_winch.disengageServo();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_winch.stop();
    m_winch.engageServo();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_winch.run(m_speed);
    return false;
  }
}
