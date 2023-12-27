package frc.robot.commands;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;

public class ClawCommand extends CommandBase {
    private ClawSubsystem m_Claw;
    private Limelight m_limelight;
    private boolean checkForTargets = false;
    private boolean m_state;
    private String m_caller;
    private static final double kClawRunTime=0.5;
    private double m_endTime;
    public ClawCommand(ClawSubsystem claw,boolean state,String caller){
        m_Claw=claw;
        m_state=state;
        m_caller=caller;
        checkForTargets=false;
    }

    public ClawCommand(ClawSubsystem claw, Limelight limelight,boolean state){
        m_Claw=claw;
        m_limelight = limelight;
        m_caller = "";
        m_state=state;
        checkForTargets = true;
    }

    @Override
    public void initialize() {
        m_endTime=Timer.getFPGATimestamp()+kClawRunTime;    
     
    }  

    @Override
    public void execute() {
        System.out.println("Setting Claw to state - " + m_state + " from caller " + m_caller);
        
        if(checkForTargets){
            if(m_limelight.isTargetAvailible()){        
                m_Claw.setClaw(m_state);
            }else{
                System.out.println("CLAW DROP LOST TARGET - DROPPING ANYWAYS");
                m_Claw.setClaw(m_state);
            }
        }else{
            m_Claw.setClaw(m_state);
        }
    }
  
    @Override
    public void end(boolean interrupted) {
    m_Claw.stopClaw();
    }
  
    @Override
    public boolean isFinished() {
     boolean returnValue=false;
     System.out.println("checking claw timer " + m_endTime + " " + Timer.getFPGATimestamp());
     returnValue=(m_endTime<Timer.getFPGATimestamp());
     return returnValue;
    }

}