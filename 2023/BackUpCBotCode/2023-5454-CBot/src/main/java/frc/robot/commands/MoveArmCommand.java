package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase {
    ArmSubsystem m_ArmSubsystem;
    double m_rotateSpeed;
    double m_minLimit;
    double m_maxLimit;
    
public MoveArmCommand(ArmSubsystem armSubsystem, double rotateSpeed, double minLimit, double maxLimit){
    m_ArmSubsystem = armSubsystem;
    m_rotateSpeed = rotateSpeed;
    m_minLimit = minLimit;
    m_maxLimit = maxLimit;
    addRequirements(m_ArmSubsystem);
}
public void initialize(){
}
@Override
public void execute(){
 //   if((m_minLimit)<(m_ArmSubsystem.getEncoderPos()) && (m_maxLimit>m_ArmSubsystem.getEncoderPos())){
 //       if(m_ArmSubsystem.getEncoderPos()<m_maxLimit)
            m_ArmSubsystem.rotateArm(m_rotateSpeed);
 //   }else{
//       System.out.println("Manual Arm Soft Limit " + m_minLimit + " " + m_ArmSubsystem.getEncoderPos() + " " + m_maxLimit);
 //       m_ArmSubsystem.stopRotate();
 //   }

    }

@Override
public void end(boolean interrupted) {
    m_ArmSubsystem.stopRotate();
}
}