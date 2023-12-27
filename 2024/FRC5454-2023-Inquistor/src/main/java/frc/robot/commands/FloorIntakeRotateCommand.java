package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class FloorIntakeRotateCommand extends CommandBase{
    private ElevatorSubsystem m_elevator;
    private FloorIntakeSubsystem m_intake;
    private Double m_speed;
    private double m_liftHeight;

    public FloorIntakeRotateCommand(FloorIntakeSubsystem intake,ElevatorSubsystem elevator,double  speed,double liftHeight){
        m_intake = intake;
        m_elevator = elevator;
        m_speed = speed;
        m_liftHeight=liftHeight;

     //Allow both flor commands to use intake since thy are using different motors
        //addRequirements(m_intake);
    
    }

    @Override
    public void execute(){
       //     System.out.println("FlrRotate " + m_speed);
             if(m_elevator.getElevatorPos() < m_liftHeight){
                    m_intake.rotate(m_speed);                
            }else{
                //have to travel past height to allow for drift down that still occurs
                m_elevator.SetPosAndMove(m_liftHeight-10);
                m_intake.stopRotate();
            }
            
        }

    @Override
    public void end(boolean interrupted){
        m_intake.stopRotate();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //STOP if we hit a limit
        if(m_intake.checkRotateLimits(m_speed)){
            m_intake.stopRotate();
            return true;
        }else {
            return false;
        }
        
    }
}
