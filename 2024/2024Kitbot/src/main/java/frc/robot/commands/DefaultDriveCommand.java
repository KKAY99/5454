package frc.robot.commands;
import java.util.Enumeration;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBase;

public class DefaultDriveCommand extends Command{

  private DriveBase m_Drive;

  private double m_ySpeed;
  private double m_xSpeed;

  public DefaultDriveCommand(DriveBase drive,DoubleSupplier leftY,DoubleSupplier leftX){
    m_Drive=drive;

    m_ySpeed=leftY.getAsDouble();
    m_xSpeed=leftX.getAsDouble();
  }

  @Override
  public void end(boolean interrupted){
    m_Drive.StopMotors();
  }

  @Override
  public boolean isFinished(){
    if(m_xSpeed<0){
      m_Drive.RunRightMotors(m_xSpeed);
      m_Drive.RunLeftMotors(0);

    }else if(m_xSpeed>0){
      m_Drive.RunLeftMotors(m_xSpeed);
      m_Drive.RunRightMotors(0);

    }else{
      m_Drive.RunRightMotors(m_ySpeed);
      m_Drive.RunLeftMotors(m_ySpeed);
    }

    return false;
  }
}
