package frc.robot.commands;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendClawCommand extends CommandBase {

    private PneumaticsSubsystem m_pneumatics;

    public ExtendClawCommand(PneumaticsSubsystem pneumatics){
        m_pneumatics=pneumatics;
    }

    @Override
    public void execute(){
        if(m_pneumatics.getExtendedState()==true){
            m_pneumatics.setExtensionSolenoid(false);
        }else{
            m_pneumatics.setExtensionSolenoid(true);
        }
    }
    @Override
    public boolean isFinished(){
        return true;
    }
     
}
