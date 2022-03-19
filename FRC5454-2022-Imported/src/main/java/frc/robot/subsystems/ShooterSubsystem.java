package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX Bottom_ShooterMotor;
  private TalonFX Top_ShooterMotor;
  private double m_VelocityMultiplier=1;
  private static double m_defaultTopSpeed=775;
  private static double m_defaultBottomSpeed=775;
  private static double[] powerTopValues = {
    775,
    775,
    775,
    775,
    825,
    825,
    925,
    1075,
    1182,
    1182,
    1182,
    1670,
    1670,
    1375,
    1475,
    25
};

private static double[] powerBottomValues = {
    775,
    775,
    775,
    775,
    825,
    825,
    925,
    1075,
    1182,
    1182,
    1182,
    1470,
    1470,
    1375,
    1475,
    1075
};

private static double[] distanceValues = {
    39.5,
    49.6,
    59.5,
    69.5,
    79.5,
    89.5,
    99.5,
    109.5,
    119.5,
    129.5,
    139.5,
    149.5,
    159.5,
    169.5,
    179.5,
    189.5,
};
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem(Integer BottomPort, Integer TopPort) {
    Bottom_ShooterMotor = new TalonFX(BottomPort);
    Top_ShooterMotor = new TalonFX(TopPort);

    Bottom_ShooterMotor.configOpenloopRamp(0.5);
    Top_ShooterMotor.configOpenloopRamp(0.5);
    
    Bottom_ShooterMotor.setInverted(false);


    Bottom_ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,30); 
    Bottom_ShooterMotor.selectProfileSlot(0,0);
    Bottom_ShooterMotor.config_kP(0,0);
    Bottom_ShooterMotor.config_kI(0,0);
    Bottom_ShooterMotor.config_kD(0,0);
    Bottom_ShooterMotor.config_kF(0,.5);
    
    Top_ShooterMotor.setInverted(true);
    Top_ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,30); 
    Top_ShooterMotor.selectProfileSlot(0,0);
    Top_ShooterMotor.config_kP(0,0);
    Top_ShooterMotor.config_kI(0,0);
    Top_ShooterMotor.config_kD(0,0);
    Top_ShooterMotor.config_kF(0,.5);
    
  }
  public double getTopMotorVelocity(){ 
    return Top_ShooterMotor.getSelectedSensorVelocity(0);
  }
  public double getBottomMotorVelocity(){
    return Top_ShooterMotor.getSelectedSensorVelocity(0);
  }
  public double getMultiplier(){
    return m_VelocityMultiplier;
  }
  public void setMultipler(double newValue){
    m_VelocityMultiplier=newValue;
  }
  public void shootbyDistance(double distance, double defaultTopSpeed, double defaultBottomSpeed){
    double topSpeed = getPower(powerTopValues, distance);
    double bottomSpeed = getPower(powerBottomValues, distance);
    //Default shooting if calculation is outside range
    if(topSpeed<=1 || bottomSpeed <=1){
      if(defaultTopSpeed==0){
        topSpeed=m_defaultTopSpeed;
        bottomSpeed=m_defaultBottomSpeed;
      }else {
        topSpeed=defaultTopSpeed;
        bottomSpeed=defaultBottomSpeed;
      }
    }
    //system.out.println("Shoot By Distance - d=" + distance + " top:"+ topSpeed + " bottom:" + bottomSpeed);
    shoot(topSpeed,bottomSpeed);
  }
  public void shoot(double topVelocity, double bottomVelocity) {
    bottomVelocity=bottomVelocity*m_VelocityMultiplier;
    topVelocity=topVelocity*m_VelocityMultiplier;
    Bottom_ShooterMotor.set(ControlMode.Velocity, bottomVelocity);
    Top_ShooterMotor.set(ControlMode.Velocity, topVelocity);
    
 
  }

  public static double getPower(double powerValues[], double distance) {
    int i=0;
    try{
      distance = Math.max(distance, 0);
      for (i = 0; i < distanceValues.length; i++) {
        if (distanceValues[i] == distance) {
          return powerValues[i];
        } else if (distanceValues[i] > distance) {
          return getEquation(distance, distanceValues[i], powerValues[i], distanceValues[i - 1],
              powerValues[i - 1]);
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
  
  public void stop() {
    Bottom_ShooterMotor.set(ControlMode.PercentOutput, 0.0);
    Top_ShooterMotor.set(ControlMode.PercentOutput, 0.0);
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
