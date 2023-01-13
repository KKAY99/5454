// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.RobotContainer;
import com.kauailabs.navx.frc.AHRS;

public class testCommand extends CommandBase {

  private ExampleSubsystem m_subsystem;
  private AHRS m_gyro;

  /** Creates a new testCommand. */
  public testCommand(AHRS gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gyro=gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem = new ExampleSubsystem();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_gyro.getRoll() <= 0.5 || m_gyro.getRoll() >= -0.5){
      m_subsystem.stop();
      return true;
     }else if(m_gyro.getRoll() > 0){
       m_subsystem.run(-0.3);
       return false;
     }else{
       m_subsystem.run(0.3);
       return false;
     }
   
  }
}