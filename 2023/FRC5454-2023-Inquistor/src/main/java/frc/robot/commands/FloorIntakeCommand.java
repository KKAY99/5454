package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class FloorIntakeCommand extends CommandBase{
    private FloorIntakeSubsystem m_intake;
    private ElevatorSubsystem m_elevator;
    private double m_intakeSpeed;
    private double m_rotateSpeed;
    private STATE m_state;
    private STATE m_origstate;
    private double m_liftHeight;
    private double kRotateTolerance=5;
    private double m_timeOut;
    private double m_startTime=0;

    public enum STATE {
        AUTOINTAKE,AUTOEXHAUST,MOVEANDINTAKE,INTAKE,EXHAUST,MOVEHOME,MOVETRANSFER,ABORT,END
    }

    public FloorIntakeCommand(FloorIntakeSubsystem intake, ElevatorSubsystem elevator, double intakeSpeed,double rotateSpeed,double liftHeight, STATE state, double timeOut){
        m_intake = intake;
        m_intakeSpeed = intakeSpeed;
        m_rotateSpeed=rotateSpeed;
        //System.out.println("ROTATE SPEED IS " + m_rotateSpeed);
        m_state=state;
        m_origstate=state;
        m_elevator=elevator;
        m_liftHeight=liftHeight;
        m_timeOut=timeOut;
        //Allow both flor commands to use intake since thy are using different motors
        //addRequirements(m_intake);
    }
    public  FloorIntakeCommand(FloorIntakeSubsystem intake, ElevatorSubsystem elevator, double intakeSpeed,double rotateSpeed,double liftHeight, STATE state)
    {
        this(intake,elevator,intakeSpeed,rotateSpeed,liftHeight,state,0);
    }
    @Override
    public void execute(){
    }
    @Override
    public void initialize(){
        m_state=m_origstate;
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopIntake();
        m_intake.stopRotate();
        m_state=STATE.END;
    }
    private boolean isClawMoved(){
        System.out.println("Is Claw Moved " + m_elevator.getElevatorPos() + "checking against" +(m_liftHeight-10));
        if(m_elevator.getElevatorPos() < m_liftHeight){
               return true;                
       }else{
           //have to travel past height to allow for drift down that still occurs
           m_elevator.SetPosAndMove(m_liftHeight-10);
           m_intake.stopRotate();
           return false;
       }
      
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double rotatePos=m_intake.getRotatePos();
      //  System.out.println(m_state + "  RotatePos:" + rotatePos);
        boolean returnValue=false;
        switch(m_state){
            case AUTOINTAKE:
            if(isClawMoved()==true){
                System.out.println("AI-" + rotatePos + " ** " + m_rotateSpeed);
              if(rotatePos<Constants.FloorIntake.rotateHandOff){
                 m_intake.rotate(m_rotateSpeed);
             }else{
                  m_state=STATE.MOVEANDINTAKE;
             }
           }
            break;
            case MOVEANDINTAKE:
            m_intake.runIntake(m_intakeSpeed);
            m_intake.rotate(m_rotateSpeed);

            if(rotatePos>=Constants.FloorIntake.rotateLowLimit){
                m_intake.stopRotate();
                m_state=STATE.INTAKE;
            }
            break;
            case INTAKE:
            //if m_timeOut=0 then running in user mode 
            if(m_timeOut==0){
                m_intake.runIntake(m_intakeSpeed);

            }else {
                double currentTime=Timer.getFPGATimestamp();
                if(m_startTime==0){
                    m_startTime=Timer.getFPGATimestamp();
                }
                if(currentTime>m_startTime+m_timeOut){
                    returnValue=true;
                }else{
                    m_intake.runIntake(m_intakeSpeed);
                }
            }  
            break;
            case AUTOEXHAUST:
            System.out.println(rotatePos + " **** " + m_intake.getRotatePos());
            if(isClawMoved()==true){
                if(rotatePos>=Constants.FloorIntake.rotateOuttakePos){
                m_intake.rotate((-m_rotateSpeed));
                System.out.println("rotate up " + m_rotateSpeed);
                }else{
                    if(rotatePos<=Constants.FloorIntake.rotateOuttakePos){
                        m_intake.rotate((m_rotateSpeed));
                    System.out.println("rotate down" + m_rotateSpeed);
                    }
                } 
            }   else{
                //stop rotate
            
                m_intake.stopRotate(); // if claw is in the way don't rotate
            
            }   
            if(Math.abs(rotatePos-Constants.FloorIntake.rotateOuttakePos)<kRotateTolerance){
                m_intake.stopRotate();
                m_state=STATE.EXHAUST;
            }
            
            break;
            case EXHAUST:
            m_intake.runIntake(-m_intakeSpeed);
            break;
            case MOVEHOME:
            if(rotatePos>=Constants.FloorIntake.rotateHighLimit){
                m_intake.rotate(-m_rotateSpeed);
            }else{
                if(rotatePos<=Constants.FloorIntake.rotateHighLimit){
                    m_intake.rotate(m_rotateSpeed); 
                }
            }
            if(rotatePos==Constants.FloorIntake.rotateHighLimit){
                m_intake.stopRotate();
                m_state=STATE.END;
            }
            break;
            case MOVETRANSFER:
            break;
            case ABORT:
            case END:
            returnValue=true;
        }      
        return returnValue;
    }
}
