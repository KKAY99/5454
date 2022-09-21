package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ShooterSubsystemVoltage implements Subsystem {
  private CANSparkMax m_Bottom_ShooterMotor;
  private CANSparkMax m_Top_ShooterMotor;
  private double m_VelocityMultiplier=1;
  private static double m_defaultTopSpeed=.4;
  private static double m_defaultBottomSpeed=.5;
  private static double kGearRatio=6;
  private static double m_PrimeSpeed;
  
  private static double[] powerTopValues = {
    .40,//1
    .40,//2
    .45,//3
    .45,//4
    .40,//5
    .40,//6
    .45,//7
    .45,//8
    .45,//9
    .45,//10
    .60,//11
    .700,//12
    .700,//13
    .700,//14
    .750,//15
    .40 //16
};

private static double[] powerBottomValues = {
    .45,//1
    .45,//2
    .50,//3
    .40,//4
    .38,//5
    .42,//6
    .45,//7
    .50,//8
    .55,//9
    .60,//10
    .800,//11
    .840,//12
    .860,//13
    .880,//14
    .900,//15
    .990 //16
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
    189.5//16
};
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystemVoltage(Integer BottomPort, Integer TopPort,double primeSpeed) {
    m_PrimeSpeed=primeSpeed;
    m_Bottom_ShooterMotor = new CANSparkMax(BottomPort, MotorType.kBrushless);
    m_Bottom_ShooterMotor.enableVoltageCompensation(12);

    m_Top_ShooterMotor = new CANSparkMax(TopPort, MotorType.kBrushless);
    m_Top_ShooterMotor.enableVoltageCompensation(12);
    
   
    m_Bottom_ShooterMotor.setInverted(true);
    //m_Bottom_ShooterMotor.setNeutralMode(NeutralMode.Coast);
    //m_Top_ShooterMotor.setNeutralMode(NeutralMode.Coast);
     /*
    m_Bottom_ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,30); 
    
    m_Bottom_ShooterMotor.selectProfileSlot(0,0);
    m_Bottom_ShooterMotor.config_kP(0,0);
    m_Bottom_ShooterMotor.config_kI(0,0);
    m_Bottom_ShooterMotor.config_kD(0,0);
    m_Bottom_ShooterMotor.config_kF(0,.5);
    */
    m_Top_ShooterMotor.setInverted(false);
    /*
    m_Top_ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,30); 
    m_Top_ShooterMotor.selectProfileSlot(0,0);
    m_Top_ShooterMotor.config_kP(0,0);
    m_Top_ShooterMotor.config_kI(0,0);
    m_Top_ShooterMotor.config_kD(0,0);
    m_Top_ShooterMotor.config_kF(0,.5);
    */
  }
  public double getTopMotorVelocity(){
    return m_Top_ShooterMotor.getEncoder().getVelocity() /5700;
    //return 0; // m_TopShooterMotor.getSelectedSensorVelocity(0)/kGearRatio;
  }
  public double getBottomMotorVelocity(){
    return m_Bottom_ShooterMotor.getEncoder().getVelocity() /5700; 
    //return 0; //m_Top_ShooterMotor.getSelectedSensorVelocity(0)/kGearRatio;
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
    double topCurrentSpeed;
    //Default shooting if calculation is outside range
    if(topSpeed<=0.1 || bottomSpeed <=0.1){
        topSpeed=m_defaultTopSpeed;
        bottomSpeed=m_defaultBottomSpeed;      
      }
      bottomSpeed=bottomSpeed*m_VelocityMultiplier;
      topSpeed=topSpeed*m_VelocityMultiplier;
      topCurrentSpeed=getTopMotorVelocity();
      System.out.println("Checking if up to speed" + topCurrentSpeed + " / " + topSpeed);    
      if ((topCurrentSpeed>=topSpeed) && (getBottomMotorVelocity()>=bottomSpeed)){
        returnvalue=true;
    } 
    System.out.println(" Up To Speed-" + returnvalue);
    
    return returnvalue;
  }

  public void shootbyDistance(double distance){
    double topSpeed = getPower(powerTopValues, distance);
    double bottomSpeed = getPower(powerBottomValues, distance);
    //Default shooting if calculation is outside range
    if(topSpeed<=0.1 || bottomSpeed <=0.1){
        topSpeed=m_defaultTopSpeed;
        bottomSpeed=m_defaultBottomSpeed;      
      }
    //system.out.println("Shoot By Distance - d=" + distance + " top:"+ topSpeed + " bottom:" + bottomSpeed);
    shoot(topSpeed,bottomSpeed);
  }
  public void shoot(double topVelocity, double bottomVelocity) {
    System.out.println("Shoooter  Raw Values " + " Bottom: " + bottomVelocity + " - Top: " + topVelocity);
    bottomVelocity=bottomVelocity*m_VelocityMultiplier;
    topVelocity=topVelocity*m_VelocityMultiplier;
    //Adjust Motor to %
    if(topVelocity>1.0){
      topVelocity=1.0;
    }
    if (bottomVelocity>1.0){
      bottomVelocity=1.0;
    }
    System.out.println("Shoooter " + " Bottom: " + bottomVelocity + "- Top: " + topVelocity);
     m_Bottom_ShooterMotor.set(bottomVelocity);

     m_Top_ShooterMotor.set(topVelocity);
    //m_Bottom_ShooterMotor.set(.50 * m_VelocityMultiplier);
   // m_Top_ShooterMotor.set(.45 * m_VelocityMultiplier);
 
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
  
  public void stopShooting() {
    m_Bottom_ShooterMotor.set( m_PrimeSpeed);
    m_Top_ShooterMotor.set( m_PrimeSpeed);
  }

  public void stopShooter(){
    m_Bottom_ShooterMotor.set(0);
    m_Top_ShooterMotor.set(0);
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
