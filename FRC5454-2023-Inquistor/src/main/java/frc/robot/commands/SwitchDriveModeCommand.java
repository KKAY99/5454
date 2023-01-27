package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.DriveControlMode;

public class SwitchDriveModeCommand extends CommandBase {
    
    private DriveControlMode m_driveControlMode;
    public SwitchDriveModeCommand(DriveControlMode driveMode){
       m_driveControlMode=driveMode;
    }

    @Override
    public void initialize() {
    }  

    @Override
    public void execute() {
        if(m_driveControlMode.isFieldOrientated()){
            m_driveControlMode.setRobotMode();
        }else{
            m_driveControlMode.setFieldMode();
        }
    }
    
  
    @Override
    public void end(boolean interrupted) {
    
    }
  
    @Override
    public boolean isFinished() {
     return true;
    }

}
