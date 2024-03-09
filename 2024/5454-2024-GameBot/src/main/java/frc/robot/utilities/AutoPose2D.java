package frc.robot.utilities;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;

public class AutoPose2D{
    private Pose2d m_pos;
    private Pose2d m_intakePos;
    private Pose2d m_shotPos;
    private boolean m_shouldTurn90;
    private boolean m_shouldShoot;

    public AutoPose2D(Pose2d pos,Pose2d intakePos,Pose2d shotPos,boolean shouldTurn90,boolean shouldShoot){
        //This constructor is for short note locations
        m_pos=pos;
        m_intakePos=intakePos;
        m_shotPos=shotPos;
        m_shouldShoot=shouldShoot;
        m_shouldTurn90=shouldTurn90;
    }

    public AutoPose2D(Pose2d pos,boolean shouldTurn90,boolean shouldShoot){
        //This constructor is for long note locations
        m_pos=pos;
        m_shouldShoot=shouldShoot;
        m_shouldTurn90=shouldTurn90;
    }

    public Pose2d getPathPos(){
        return m_pos;
    }

    public Pose2d getIntakePos(){
        Pose2d returnValue=null;

        if(m_intakePos==null){
            returnValue=FindLongIntakePos();
        }else{
            returnValue=m_intakePos;
        }
        return returnValue;
    }

    public Pose2d getShotPos(){
        Pose2d returnValue=null;

        if(m_shotPos==null){
            returnValue=FindLongShotPos();
        }else{
            returnValue=m_shotPos;
        }
        return returnValue;
    }

    public boolean shouldShootNote(){
        return m_shouldShoot;
    }

    public double getIntakeRotatePose(){
        double returnValue=0;

        if(m_shouldTurn90){
            returnValue=TurretConstants.turret90Pos;
        }else{
            returnValue=TurretConstants.turretStraightPos;
        }

        return returnValue;
    }

    private Pose2d FindLongShotPos(){
        Pose2d returnValue=null;

        if(m_pos==AutoConstants.locationLongAmpNote||m_pos==AutoConstants.locationLongAmp2Note){
            if(DriverStation.getAlliance().get()==Alliance.Red){
                returnValue=AutoConstants.locationRedLongAmpWing;
            }else{
                returnValue=AutoConstants.locationBlueLongAmpWing;
            }
        }

        if(m_pos==AutoConstants.locationLongSourceNote||m_pos==AutoConstants.locationLongSource2Note||m_pos==AutoConstants.locationLongCenterNote){
            if(DriverStation.getAlliance().get()==Alliance.Red){
                returnValue=AutoConstants.locationRedLongSourceWing;
            }else{
                returnValue=AutoConstants.locationBlueLongSourceWing;
            }
        }

        return returnValue;
    }

    private Pose2d FindLongIntakePos(){
        Pose2d returnValue=null;

        if(m_pos==AutoConstants.locationLongAmpNote){
            if(DriverStation.getAlliance().get()==Alliance.Red){
                returnValue=AutoConstants.longAmpNoteRedIntakeWaypoint;
            }else{
                returnValue=AutoConstants.longAmpNoteBlueIntakeWaypoint;
            }
        }

        if(m_pos==AutoConstants.locationLongAmp2Note){
            if(DriverStation.getAlliance().get()==Alliance.Red){
                returnValue=AutoConstants.longAmp2NoteRedIntakeWaypoint;
            }else{
                returnValue=AutoConstants.longAmp2NoteBlueIntakeWaypoint;
            }
        }

        if(m_pos==AutoConstants.locationLongCenterNote){
            if(DriverStation.getAlliance().get()==Alliance.Red){
                returnValue=AutoConstants.longCenterNoteRedIntakeWaypoint;
            }else{
                returnValue=AutoConstants.longCenterNoteBlueIntakeWaypoint;
            }
        }

        if(m_pos==AutoConstants.locationLongSourceNote){
            if(DriverStation.getAlliance().get()==Alliance.Red){
                returnValue=AutoConstants.longSourceNoteRedIntakeWaypoint;
            }else{
                returnValue=AutoConstants.longSourceNoteBlueIntakeWaypoint;
            }
        }

        if(m_pos==AutoConstants.locationLongSource2Note){
            if(DriverStation.getAlliance().get()==Alliance.Red){
                returnValue=AutoConstants.longSource2NoteRedIntakeWaypoint;
            }else{
                returnValue=AutoConstants.longSource2NoteBlueIntakeWaypoint;
            }
        }

        return returnValue;
    }
}
