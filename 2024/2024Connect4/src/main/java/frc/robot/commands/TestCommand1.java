package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestSubsystem1;
import edu.wpi.first.wpilibj.Timer;


public class TestCommand1 extends Command{
private TestSubsystem1 m_subsystem;
    private double m_startTime;

   public TestCommand1(TestSubsystem1 sub){
    m_subsystem=sub;
   } 
   
   @Override
   public void initialize() {
       m_startTime=Timer.getFPGATimestamp();
   }

   @Override
   public void execute(){
    m_subsystem.run(0.4);
   }

   @Override
   public void end(boolean interrupted){
    m_subsystem.stop();

   }

   @Override
   public boolean isFinished() {
    boolean returnValue=false;

        if(Timer.getFPGATimestamp()>m_startTime+1){
            returnValue=true;
        } 
        
        return returnValue;
    }
}