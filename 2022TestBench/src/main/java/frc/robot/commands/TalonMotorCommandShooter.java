// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TalonMotorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/** An example command that uses an example subsystem. */
public class TalonMotorCommandShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TalonMotorSubsystem m_subsystem1;
  private final TalonMotorSubsystem m_subsystem2;
  private double m_speed;
  private String m_SmartDashboardKey;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TalonMotorCommandShooter(TalonMotorSubsystem subsystem1,TalonMotorSubsystem subsystem2,double speed) {
    m_subsystem1 = subsystem1;
    m_subsystem2 = subsystem2;
    m_speed=speed;
   // m_SmartDashboardKey=SmartDashboardKey;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem1);
    addRequirements(subsystem2);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    m_subsystem1.setSpeed(m_speed);
    m_subsystem2.setSpeed(0-m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem1.setSpeed(0);
    m_subsystem2.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
