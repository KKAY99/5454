package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ShooterSubsystem implements Subsystem {
  private TalonFX m_Bottom_ShooterMotor;
  private TalonFX m_Top_ShooterMotor;
  private double m_VelocityMultiplier=1;
  private static double m_defaultTopSpeed=775;
  private static double m_defaultBottomSpeed=775;
  private static double kGearRatio=6;
  private double m_TopPrimeSpeed;
  private double m_BottomPrimeSpeed;
  
  private static double[] powerTopValues = {
    775,//1
    775,//2
    775,//3
    775,//4
    866,//5
    875,//6
    1000,//7
    1102,//8
    1152,//9
    1400,//10
    1550,//11
    1700,//12
    1800,//13
    1900,//14
    2000,//15
    500, //16
    1800 //17
};

private static double[] powerBottomValues = {
    775,//1
    775,//2
    775,//3
    775,//4
    866,//5
    875,//6
    1000,//7
    1102,//8
    1252,//9
    2150,//10
    2150,//11
    1700,//12
    1800,//13
    1900,//14
    2000,//15
    500,//16
    1121//17
};

public static double[] distanceValues = {
    39.5, //1
    49.6, //2
    59.5, //3
    69.5, //4
    79.5,//5
    89.5,//6
    99.5,//7
    109.5,//8
    119.5,//9
    129.5,//10
    139.5,//11
    149.5,//12
    159.5,//13
    169.5,//14
    179.5,//15
    180.5,//16
    181.5//17
};
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem(Integer BottomPort, Integer TopPort,double primeSpeed) {
    m_TopPrimeSpeed=primeSpeed;
    m_BottomPrimeSpeed=primeSpeed;
    m_Bottom_ShooterMotor = new TalonFX(BottomPort);
    m_Top_ShooterMotor = new TalonFX(TopPort);
    
    m_Bottom_ShooterMotor.configOpenloopRamp(0.5);
    m_Top_ShooterMotor.configOpenloopRamp(0.5);
    
    m_Bottom_ShooterMotor.setInverted(false);
    m_Bottom_ShooterMotor.setNeutralMode(NeutralMode.Coast);
    m_Top_ShooterMotor.setNeutralMode(NeutralMode.Coast);

    m_Bottom_ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,30); 
    m_Bottom_ShooterMotor.selectProfileSlot(0,0);
    m_Bottom_ShooterMotor.config_kP(0,0);
    m_Bottom_ShooterMotor.config_kI(0,0);
    m_Bottom_ShooterMotor.config_kD(0,0);
    m_Bottom_ShooterMotor.config_kF(0,.5);
    
    m_Top_ShooterMotor.setInverted(true);
    m_Top_ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,30); 
    m_Top_ShooterMotor.selectProfileSlot(0,0);
    m_Top_ShooterMotor.config_kP(0,0);
    m_Top_ShooterMotor.config_kI(0,0);
    m_Top_ShooterMotor.config_kD(0,0);
    m_Top_ShooterMotor.config_kF(0,.5);
    
  }
  public double getTopMotorVelocity(){ 
    return m_Top_ShooterMotor.getSelectedSensorVelocity(0)/kGearRatio;
  }
  public double getBottomMotorVelocity(){
    return m_Top_ShooterMotor.getSelectedSensorVelocity(0)/kGearRatio;
  }
  public double getMultiplier(){
    return m_VelocityMultiplier;
  }
  public void setMultipler(double newValue){
    m_VelocityMultiplier=newValue;
  }
  
  public boolean isUptoSpeedbyDistance(double distance){
    boolean returnvalue=false;
    double topSpeed = getPower(powerTopValues, distance);
    double bottomSpeed = getPower(powerBottomValues, distance);
    
    //Default shooting if calculation is outside range
    if(topSpeed<=1 || bottomSpeed <=1){
        topSpeed=m_defaultTopSpeed;
        bottomSpeed=m_defaultBottomSpeed;      
      }
      bottomSpeed=bottomSpeed*m_VelocityMultiplier;
      topSpeed=topSpeed*m_VelocityMultiplier;
      if ((getTopMotorVelocity()>=topSpeed) && (getBottomMotorVelocity()>=bottomSpeed)){
        returnvalue=true;
    }
    return returnvalue;
  }

  public void shootbyDistance(double distance){
    double topSpeed = getPower(powerTopValues, distance);
    double bottomSpeed = getPower(powerBottomValues, distance);
    //Default shooting if calculation is outside range
    if(topSpeed<=1 || bottomSpeed <=1){
        topSpeed=m_defaultTopSpeed;
        bottomSpeed=m_defaultBottomSpeed;      
      }
    //system.out.println("Shoot By Distance - d=" + distance + " top:"+ topSpeed + " bottom:" + bottomSpeed);
    shoot(topSpeed,bottomSpeed);
  }
  public void shoot(double topVelocity, double bottomVelocity) {
    bottomVelocity=bottomVelocity*m_VelocityMultiplier;
    topVelocity=topVelocity*m_VelocityMultiplier;
    m_Bottom_ShooterMotor.set(ControlMode.Velocity, bottomVelocity);
    m_Top_ShooterMotor.set(ControlMode.Velocity, topVelocity);
    
 
  }

  public static double getPower(double powerValues[], double distance) {
    int i=0;
    try{
      distance = Math.max(distance, 0);
      for (i = 0; i < distanceValues.length; i++) {
        if (distanceValues[i] == distance) {
          return powerValues[i];
        } else if (distanceValues[i] > distance) {
          if(i>0){
            return getEquation(distance, distanceValues[i], powerValues[i], distanceValues[i - 1],
                powerValues[i - 1]);
          }else {
            return powerValues[0];
          }  
        }
        else if (distance > distanceValues[distanceValues.length-1]) {          
          return powerValues[distanceValues.length-1];
        }
      }
      return 0;
    } catch(Exception e){
      System.out.println("Exception Error in getpower value i (" + i +  ") " + e.getMessage());
      return 0;
    }
  }

  private static double getEquation(double value, double xOne, double yOne, double xTwo, double yTwo) {
    double slope = (yTwo - yOne) / (xTwo - xOne);
    return (slope * (value - xOne)) + yOne;
  }

  public void UpdateIdleSpeed(double distance){
    if(distance > 0){
      double topSpeed = getPower(powerTopValues, distance);
      double bottomSpeed = getPower(powerBottomValues, distance);
      //Default shooting if calculation is outside range
      if(topSpeed<=1 || bottomSpeed <=1){
          topSpeed=m_defaultTopSpeed;
          bottomSpeed=m_defaultBottomSpeed;      
      }
    //  m_TopPrimeSpeed = topSpeed;
    //  m_BottomPrimeSpeed = bottomSpeed;
    //  m_Bottom_ShooterMotor.set(ControlMode.Velocity, m_TopPrimeSpeed);
    //  m_Top_ShooterMotor.set(ControlMode.Velocity, m_BottomPrimeSpeed);
    }
  }
  
  public void stopShooting() {
    m_Bottom_ShooterMotor.set(ControlMode.Velocity, m_TopPrimeSpeed);
    m_Top_ShooterMotor.set(ControlMode.Velocity, m_BottomPrimeSpeed);
  }

  public void stopShooter(){
    m_Bottom_ShooterMotor.set(ControlMode.PercentOutput,0);
    m_Top_ShooterMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}