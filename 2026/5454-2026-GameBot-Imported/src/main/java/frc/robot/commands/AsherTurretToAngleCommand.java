package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class AsherTurretToAngleCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private double m_targetAngle;
  private TurretSubsystemPots m_turret;

  private enum States{
    INIT,SEARCH,END
  }
  private States m_turretState;
  private final double kTurretFastSpeed=0.05454;
  private final double kTurretSlowSpeed=0.03;

  public AsherTurretToAngleCommand(Double targetAngle,TurretSubsystemPots turret) {
    m_targetAngle=targetAngle;
    m_turret=turret;

    m_turretState=States.INIT;

    addRequirements(m_turret);
  }

  private double POTStoAngle(double pots){
    double angle=(pots-TurretConstants.TurretLeftLimitPOTS)*36;
    return angle;
  }

  private boolean atTargetAngle(){
    double gap=Math.abs(m_targetAngle-POTStoAngle(m_turret.getTurretPOTS()));
    if(gap<3){
      return true;
    } else {
      return false;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.stopTurret();
    System.out.println("Turret Moving to " + m_targetAngle + " -> In State: " + m_turretState);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Turret Moving to " + m_targetAngle + "-> At Angle:" + POTStoAngle(m_turret.getTurretPOTS()) + " -> In State: " + m_turretState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending Turret Motion");
    m_turret.stopTurret();
  }

  
  private void gotoAngle(){
      double lorrGap=m_targetAngle-POTStoAngle(m_turret.getTurretPOTS());
      double gap=Math.abs(m_targetAngle-POTStoAngle(m_turret.getTurretPOTS()));
      double turretSpeed;
      if(gap>8){
         turretSpeed=kTurretFastSpeed;
      }else {
        turretSpeed=kTurretSlowSpeed;
      }
      if(atTargetAngle()){
        System.out.println("Turret Within Range @ Angle:" + POTStoAngle(m_turret.getTurretPOTS()));
        m_turretState=States.END;
        m_turret.stopTurret();
      } else {
        if (lorrGap<0){
          if(m_turret.getTurretPOTS()<TurretConstants.TurretRightLimitPOTS){
            //Move Left
              SmartDashboard.putString("Turret Status:", "Moving Left");
            m_turret.moveTurret(turretSpeed);
            } else {
               SmartDashboard.putString("Turret Status:", "@ Left Limit");
               m_turret.stopTurret();
            }
        } else{
           //Move Right
           System.out.println("Move R");
           if(m_turret.getTurretPOTS()>Constants.TurretConstants.TurretLeftLimitPOTS){
            SmartDashboard.putString("Turret Status:", "Moving Right");
            m_turret.moveTurret(-turretSpeed);
          } else {
            SmartDashboard.putString("Turret Status:", "@ Right Limit");
            m_turret.stopTurret();
          }
       }
      }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    switch(m_turretState){
      case INIT:
        m_turret.stopTurret();
        if(atTargetAngle()){
          m_turretState=States.END;
        } else {
          m_turretState=States.SEARCH;
        }
        break;
      case SEARCH:
        gotoAngle();
        break;
      case END:
        returnValue=true;
        break;
    } 
    return returnValue;
  }
}
