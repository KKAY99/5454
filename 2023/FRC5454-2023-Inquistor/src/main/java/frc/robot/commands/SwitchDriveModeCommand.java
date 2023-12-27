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
            //System.out.println("Switching Drive to Robot Centric Mode");
            m_driveControlMode.setRobotMode();
        }else{
        //System.out.println("Switching Drive to Field Centric Mode");
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
