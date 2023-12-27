package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.SparkMaxAlternateEncoder;
import frc.robot.subsystems.LiftSubsystem;

public class LiftBrakeCommand  extends CommandBase{
    private LiftSubsystem m_liftSubsystem;
    private SparkMaxPIDController m_pidController;
    private double m_targetPos;
    public LiftBrakeCommand(LiftSubsystem liftSubsystem){
        m_liftSubsystem = liftSubsystem;
    }

    @Override
    public void initialize(){
        m_targetPos = m_liftSubsystem.GetPos()-45;
        System.out.println(m_targetPos);
       
    } 

    @Override
    public void execute(){
        m_liftSubsystem.SetPosAndMove(m_targetPos);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
