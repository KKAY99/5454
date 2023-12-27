// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.RotateArm;
import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final RotateArmSubsystem m_RotateSubsystem;
  private final DoubleSupplier m_speed;
  private final Double m_maxValue;
  private final double kRequiredRotateAngle=0.24;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCommand(ElevatorSubsystem elevator,RotateArmSubsystem rotate,DoubleSupplier speedSupplier,double maxValue) {
    m_ElevatorSubsystem  = elevator;
    m_RotateSubsystem=rotate;
    m_speed=speedSupplier;
    m_maxValue=maxValue;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed=m_speed.getAsDouble();
    if(speed>m_maxValue){
      speed=m_maxValue;
    }else{
      if(speed<0-m_maxValue){
        speed=0-m_maxValue;
        }
    }
    //enfoce height limit only when moving up
    //TODO: Need to add in to enforce limit
    /*   if(speed<0){
      if (m_RotateSubsystem.getAbsolutePos()<kRequiredRotateAngle){
        speed=0;
      }
    }
  */
 //   m_ElevatorSubsystem.runWithOutLimit(speed);
    
    m_ElevatorSubsystem.runWithLimit(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
