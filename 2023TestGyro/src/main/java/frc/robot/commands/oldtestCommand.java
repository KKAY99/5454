// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.RobotContainer;
import com.kauailabs.navx.frc.AHRS;

public class oldtestCommand extends CommandBase {

  private ExampleSubsystem m_subsystem;
  private AHRS m_gyro;
  private double zerolevel =-.77;
  
  /** Creates a new testCommand. */
  public oldtestCommand(AHRS gyro,ExampleSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gyro=gyro;
    m_subsystem=subsystem;
  
    addRequirements(subsystem);
  }

   public void setLevel(double level){
     zerolevel =level;
  
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Execute Command - " + zerolevel + " * CurRoll " + m_gyro.getRoll());
    

    
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("IsFinished Command" + m_gyro.getRoll());
    if(m_gyro.getRoll() >= (zerolevel-0.05) &&  m_gyro.getRoll() <= (zerolevel+0.05)){
      System.out.println("stop");
      m_subsystem.stop();
      return true;
     }else if(m_gyro.getRoll() > zerolevel){
       System.out.println("go forward");
       m_subsystem.run(0.22);
       return false;
     }else{
      System.out.println("go back");
       m_subsystem.run(-0.22);
       return false;
     }
   
  }
}