package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class OpenBottomClawCommand extends CommandBase{

    private PneumaticsSubsystem m_pneumatics;

    public OpenBottomClawCommand(PneumaticsSubsystem pneumatics){
        m_pneumatics=pneumatics;
    }

    @Override
    public void execute(){
        System.out.println("This is a bottom claw");
        if(m_pneumatics.getBottomClawSolenoidState()==true){
            m_pneumatics.setBottomClawSolenoid(false);
        }else{
            m_pneumatics.setBottomClawSolenoid(true);
        }
    }
    @Override
    public boolean isFinished(){
        return true;
    }
    
}
