package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class zPivotAndExtend extends CommandBase {
    public Constants.TargetHeight m_height;
    public zPivotAndExtend(Constants.TargetHeight height){
        m_height = height;
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
    
    }
  
    @Override
    public void end(boolean interrupted) {
    
    }
  
    @Override
    public boolean isFinished() {
     return false;
    }
}
