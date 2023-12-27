package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimitSwitches;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
public class TestSwerveModuleSubsystem extends SubsystemBase{
    private CANSparkMax m_turningMotor;
    private CANSparkMax m_driveMotor;
    public TestSwerveModuleSubsystem(int driveMotorPort,int turnMotorPort,int encoderPort){ 
        m_turningMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        m_turningMotor.setSmartCurrentLimit(30);  
        m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_driveMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);
        m_driveMotor.setSmartCurrentLimit(30);  
        m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
               
    }
    public void drive(double drive,double turn){
      driveMotor(drive);
      turn(turn);
    }
    public void driveMotor(double speed){
      m_driveMotor.set(speed);
    }

    public void turn (double speed){
      m_turningMotor.set(speed);
    }
    
    public void stopDrive(){
      m_driveMotor.set(0);
    }
     public void stopTurn(){ 
      m_turningMotor.set(0);
    }

    public double getDriveSpeed(){
      return m_driveMotor.get();
    }
    public double getTurnSpeed(){
      return m_turningMotor.get();
    }
    public double getAngleCanCoder(){
      return 99;
    }
    public double getTurnEncoder(){
      return 300;
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
//     System.out.println(" * Pos  " + getRotatePos() + " " + getLimitSwitch());
    }
}
