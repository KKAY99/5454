package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.SETPOSPOSITIONS;

public class SetPositionCommand extends CommandBase{

    public SETPOSPOSITIONS m_position;
    public boolean m_shouldPivotUp=false;
    public boolean m_shouldOpenClawTop=false;
    public boolean m_activateHoldSolenoid=false;
    public RotateArmSubsystem m_Rotate;
    public PneumaticsSubsystem m_Pneumatics;
    public double m_targetPos;

    private enum STATE{
        ROTATE,HOLDSOLENOID,PIVOT,OPENTOPCLAW,END
    }

    private STATE m_state=STATE.HOLDSOLENOID;

    public SetPositionCommand(RotateArmSubsystem rotate, PneumaticsSubsystem pneumatics, SETPOSPOSITIONS position){
        m_position=position;
        m_Rotate=rotate;
        m_Pneumatics=pneumatics;
    }

    @Override
    public void initialize() {
      m_state=STATE.HOLDSOLENOID;

      switch(m_position){
        case STARTING:
        m_targetPos=Constants.Rotate.autoRotateStarting;
        m_activateHoldSolenoid=false;
        m_shouldPivotUp=false;
        m_shouldOpenClawTop=false;
        break;
        case SCORE:
        m_targetPos=Constants.Rotate.autoRotateScore;
        m_shouldPivotUp=true;
        m_shouldOpenClawTop=false;
        m_activateHoldSolenoid=false;
        break;
        case PLAYERSTATION:
        m_targetPos=Constants.Rotate.autoRotatePlayerStation;
        m_activateHoldSolenoid=true;
        m_shouldPivotUp=false;
        m_shouldOpenClawTop=true;

        break;
    }
    }

    @Override
    public boolean isFinished(){
        switch(m_state){
            case HOLDSOLENOID:
            System.out.println("Holding Solenoid state");
            if(m_activateHoldSolenoid){
                System.out.println("solenoid state to true");
                m_Pneumatics.setArmHoldCynlinder(true);
                m_activateHoldSolenoid=false;
            }else{
                m_state=STATE.ROTATE;
                if(m_targetPos!=Constants.Rotate.autoRotatePlayerStation){
                    //disabling auto
                    // m_Pneumatics.setArmHoldCynlinder(false);
                }
            }
            break;
            case ROTATE:
            if(m_targetPos>m_Rotate.getABSPos()){
                    m_Rotate.runWithOutLimits(-0.4);
            }else{
                    m_Rotate.runWithOutLimits(0.4);
            }
    
            if(m_targetPos<=m_Rotate.getABSPos()+0.004&&m_targetPos>=m_Rotate.getABSPos()-0.004){
                m_Rotate.stop();
                m_state=STATE.PIVOT;
                System.out.println("ROTATE FINISHED");
            }
            break;

            case PIVOT:
            if(m_shouldPivotUp){
                m_Pneumatics.setExtensionSolenoid(true);
                m_shouldPivotUp=false;
            }else{
                if(m_targetPos!=Constants.Rotate.autoRotateScore){
                    m_Pneumatics.setExtensionSolenoid(false);
                }
                m_state=STATE.OPENTOPCLAW;
                System.out.println("PIVOT FINSIHED");
            }

            break;
            case OPENTOPCLAW:
            if(m_shouldOpenClawTop){
                m_Pneumatics.setBottomClawSolenoid(true);
                m_shouldOpenClawTop=false; 
            }else{
                if(m_targetPos!=Constants.Rotate.autoRotatePlayerStation){
                    m_Pneumatics.setBottomClawSolenoid(false);
                }
                m_state=STATE.END;
                System.out.println("OPENTOPCLAW FINISHED");
            }

            break;
            case END:
            System.out.println("ENDING SET POS");
            return true;
        }
        return false;
    }
        
}
