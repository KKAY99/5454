package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCubeCommand extends CommandBase{
    ShooterSubsystem m_ShooterSubsystem;
    double m_intakeMotorSpeed;
    
public IntakeCubeCommand(ShooterSubsystem shooterSubsystem,double intakeMotorSpeed){
    m_ShooterSubsystem=shooterSubsystem;
    m_intakeMotorSpeed=intakeMotorSpeed;
    addRequirements(shooterSubsystem);
   
}
public void initialize(){
   
}

@Override
public boolean isFinished(){
  boolean returnValue=false;
    returnValue=m_ShooterSubsystem.hasCube(); 
    System.out.println("Intake Running" + returnValue);
    return returnValue;
}
@Override
public void end(boolean interrupted) {
    m_ShooterSubsystem.stop();
}

@Override
public void execute(){
    m_ShooterSubsystem.intakeCube(m_intakeMotorSpeed);
   
}
}