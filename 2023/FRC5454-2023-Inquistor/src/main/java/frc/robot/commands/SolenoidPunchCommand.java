package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PnuematicsSubystem;

public class SolenoidPunchCommand extends CommandBase{
    
    private PnuematicsSubystem m_pnuematics;

    public SolenoidPunchCommand(PnuematicsSubystem pnuematics){
        m_pnuematics = pnuematics;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute(){
        if(m_pnuematics.getConveyorPunch()){
            m_pnuematics.setConveyorPunch(false);
        } else{
            m_pnuematics.setConveyorPunch(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
