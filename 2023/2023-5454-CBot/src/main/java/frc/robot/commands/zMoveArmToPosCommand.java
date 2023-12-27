package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;

public class zMoveArmToPosCommand extends CommandBase {
    double m_targetPos;
    ArmSubsystem m_ArmSubsystem;

     
    public zMoveArmToPosCommand(ArmSubsystem armSubsystem, double shootPos){
        m_ArmSubsystem=armSubsystem;
        m_targetPos=shootPos;
        addRequirements(m_ArmSubsystem);
    }
    @Override
    public void execute() {
        
    }
    @Override
    public boolean isFinished(){
        return m_ArmSubsystem.goToPos(m_targetPos);
    }
    @Override
    public void end(boolean interrupted) {
      m_ArmSubsystem.stopRotate();
    }
}