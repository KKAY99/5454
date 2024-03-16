package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.TestSubsystem1;
public class Commander extends Command{
    private TestSubsystem1 m_testSub1;
    private TestSubsystem1 m_testSub2;

    private enum STATE{SPARK1,SPARK1WAIT,SPARK2,SPARK2WAIT}

    private STATE m_states=STATE.SPARK1;

    private TestCommand1 m_testCommand;

    public Commander(TestSubsystem1 testSub, TestSubsystem1 testSub2){
        m_testSub1=testSub;
        m_testSub2=testSub2;
    }

    @Override
    public boolean isFinished() {
        switch(m_states) {
            case SPARK1:
            m_testCommand=new TestCommand1(m_testSub1);
            CommandScheduler.getInstance().schedule(m_testCommand);

            m_states=STATE.SPARK1WAIT;    
            break;
            case SPARK1WAIT:
            if(!CommandScheduler.getInstance().isScheduled(m_testCommand)){
                m_states=STATE.SPARK2;    
            }
            break;
            case SPARK2:
            m_testCommand=new TestCommand1(m_testSub2);
             CommandScheduler.getInstance().schedule(m_testCommand);
                
            m_states=STATE.SPARK2WAIT; 
            break;
            case SPARK2WAIT:
            if(!CommandScheduler.getInstance().isScheduled(m_testCommand)){
                m_states=STATE.SPARK1;    
            } 
            break;
            CommandScheduler.getInstance().rem
        }

        System.out.println("Current State: "+m_states);
        return false;
    }
}
