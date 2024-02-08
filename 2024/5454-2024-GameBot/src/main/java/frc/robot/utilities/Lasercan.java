package frc.robot.utilities;
import org.littletonrobotics.junction.Logger;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants.LaserCanConstants;
import frc.robot.subsystems.IntakeSubsystemIO;
import frc.robot.subsystems.IntakeSubsystemIOInputsAutoLogged;

public class Lasercan{
    private LasercanIO m_laserCanIO;
    private LasercanIOInputsAutoLogged m_laserCanAutoLogged=new LasercanIOInputsAutoLogged();

    private LaserCan m_highTurret;
    private LaserCan m_lowTurret;

    public Lasercan(int lowTurretid,int highTurretid){
        m_lowTurret=new LaserCan(lowTurretid);
        m_highTurret=new LaserCan(highTurretid);
    }

    public double GetDistanceInMMLow(){
        double returnValue=0.0;

        if(m_lowTurret.getMeasurement()==null){
            returnValue=0.0;
        }else{
            returnValue=m_lowTurret.getMeasurement().distance_mm;
        }

        return returnValue;
    }

    public double GetDistanceInMMHigh(){
        double returnValue=0.0;

        if(m_lowTurret.getMeasurement()==null){
            returnValue=0.0;
        }else{
            returnValue=m_highTurret.getMeasurement().distance_mm;
        }

        return returnValue;
    }

    public boolean LowTurretBreakBeam(){
        boolean returnValue=false;

        if(GetDistanceInMMLow()<LaserCanConstants.distanceToReflectorLow+LaserCanConstants.deadBand){
            returnValue=true;
        }

        return returnValue;
    }

    public boolean HighTurretBreakBeam(){
        boolean returnValue=false;

        if(GetDistanceInMMHigh()<LaserCanConstants.distanceToReflectorHigh+LaserCanConstants.deadBand){
            returnValue=true;
        }

        return returnValue;
    }

    public void laserCanPeriodic(){
        m_laserCanIO.updateInputs(m_laserCanAutoLogged);

        Logger.processInputs("LaserCan",m_laserCanAutoLogged);
        Logger.recordOutput("LowTurretBreakBeam",LowTurretBreakBeam());
        Logger.recordOutput("HighTurretBreakBeam",HighTurretBreakBeam());
    } 
}