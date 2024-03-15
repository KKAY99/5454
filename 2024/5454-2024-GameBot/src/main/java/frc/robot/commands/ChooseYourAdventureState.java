package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
public class ChooseYourAdventureState extends Command {
    private enum PositionTargets {FIRSTPOS,SECONDPOS,THIRDPOS,FOURTHPOS,FIFTHPOS,NOMOREPOS};
    private enum PositionStates {START,GOTOINTAKEPOS,TURNINTAKEON,GOTOGATHERPOS,TURNINTAKEOFF,GOTOLAUNCHPOS,SHOOT,END};
    PositionTargets m_positionTarget;
    PositionStates m_positionState;
    public ChooseYourAdventureState(){
    }
    /*
    @Override
    public void initialize(){
        m_positionTarget=PositionTargets.FIRSTPOS;
        m_positionState=PositionStates.START;
     
    }
    private PositionTargets getNextPosition(){
        return PositionTargets.NOMOREPOS;
    }
    @Override
    public boolean isFinished(){
    public boolean endAuto=false;
    switch(m_positionTarget):
        case FIRSTPOS:
        //should i go to first pos - if NOT set endAuto=true
            //collect waypoints and set the POS Varaible

        break;
        case NOMOREPOS:
            endAuto=true;
            break;
    }
    if endAuto=false{
        switch(m_positionState):
        case PositionStates.GOTOINTAKEPOS:
            //get current pos from vision
            //redfine position

        case PositionStates.END:
           m_positionState=PositionStates.START;
           m_positionTarget=getNextPosition(); //GO TO NEXT ENUM
    }
    return endAut;
     */
}
