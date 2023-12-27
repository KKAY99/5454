// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RotateCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final RotateArmSubsystem m_RotateArm;
  private final DoubleSupplier m_speed;
  private final Double m_maxValue;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateCommand(RotateArmSubsystem rotate,DoubleSupplier speedSupplier,double maxValue) {
    m_RotateArm  = rotate;
    m_speed=speedSupplier;
    m_maxValue=maxValue;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_RotateArm);
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
  
    //speed=Math.max(speed,m_maxValue);
   // speed=Math.min(speed,-m_maxValue);
    //System.out.println(m_speed.getAsDouble() + " " + speed);
    m_RotateArm.rotate(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_RotateArm.stopRotate();
    //FIX: REMOVE
    m_RotateArm.doPIDTest();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
