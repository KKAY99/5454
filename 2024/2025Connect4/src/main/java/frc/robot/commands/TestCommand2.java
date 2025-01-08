package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestSubsystem2;


public class TestCommand2 extends Command{
private TestSubsystem2 m_subsystem;
   public TestCommand2(TestSubsystem2 sub){
    m_subsystem=sub;
   }    

   @Override
   public void execute(){
    m_subsystem.run(0.8);
   }
   
   @Override
   public void end(boolean interrupted){
    m_subsystem.stop();

   }
}
