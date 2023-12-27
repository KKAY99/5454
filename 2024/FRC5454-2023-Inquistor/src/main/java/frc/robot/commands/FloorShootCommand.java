package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class FloorShootCommand extends CommandBase{
    private FloorIntakeSubsystem m_intake;
    private double m_speed;
    private double m_angle;
    private double m_shootingTime;
    private double m_startTime;
    private boolean m_hasAngle;

    public FloorShootCommand(FloorIntakeSubsystem intake,double speed, double shootingTime){
        m_intake = intake;
        m_speed = speed;
        m_shootingTime = shootingTime;
        m_hasAngle = false;

        addRequirements(m_intake);
    }

    public FloorShootCommand(FloorIntakeSubsystem intake,double speed, double shootingTime, double angle){
        m_intake = intake;
        m_speed = speed;
        m_angle = angle;
        m_shootingTime = shootingTime;
        m_hasAngle = true;

        addRequirements(m_intake);
    }

    @Override
    public void execute(){
        m_startTime = Timer.getFPGATimestamp();
        if(m_hasAngle){
           // m_intake.setRotatePos(m_angle);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        if(m_intake.getRotatePos() == m_angle){
            double currentTime = Timer.getFPGATimestamp();
            if(currentTime<m_startTime+m_shootingTime){
                m_intake.runIntake(m_speed);
            }else{
                m_intake.stopIntake();
                return true;
            }
        }
        return false;
    }
}
