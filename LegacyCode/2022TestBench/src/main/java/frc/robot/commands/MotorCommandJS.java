// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math.*;

/** An example command that uses an example subsystem. */
public class MotorCommandJS extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorSubsystem m_subsystem;
  private DoubleSupplier m_speed;
  private String m_SmartDashboardKey;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
 
   
  public MotorCommandJS(MotorSubsystem subsystem, DoubleSupplier speed,String smartDashboardKey ){
    m_subsystem=subsystem;
    m_speed=speed;
    m_SmartDashboardKey=smartDashboardKey;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    speed=m_speed.getAsDouble();
    //Deadband
    if (Math.abs(speed)<=Constants.joystickDeadband){
      speed=0;
    }
    SmartDashboard.putNumber(m_SmartDashboardKey,speed);
    m_subsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
