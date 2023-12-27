// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.RobotContainer;
import com.kauailabs.navx.frc.AHRS;

public class testCommand extends CommandBase {

  private ExampleSubsystem m_subsystem;
  private AHRS m_gyro;
  private double zerolevel =-.77;
  
  /** Creates a new testCommand. */
  public testCommand(AHRS gyro,ExampleSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gyro=gyro;
    m_subsystem=subsystem;
  
    addRequirements(subsystem);
  }

   public void setLevel(double level){
     zerolevel =level;
  
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Execute Command - " + zerolevel + " * CurRoll " + m_gyro.getRoll());
    

    
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

      double kOffBalanceAngleThresholdDegrees = 10;
      double kOonBalanceAngleThresholdDegrees  = 5;


        double xAxisRate            = 0;
        double yAxisRate            = 0;
        double pitchAngleDegrees    = m_gyro.getPitch();
        double rollAngleDegrees     = m_gyro.getRoll();
        boolean autoBalanceXMode=true;
        boolean autoBalanceYMode=true;

        if ( !autoBalanceXMode && 
             (Math.abs(pitchAngleDegrees) >= 
              Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = true;
        }
        else if ( autoBalanceXMode && 
                  (Math.abs(pitchAngleDegrees) <= 
                   Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = false;
        }
        if ( !autoBalanceYMode && 
             (Math.abs(pitchAngleDegrees) >= 
              Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = true;
        }
        else if ( autoBalanceYMode && 
                  (Math.abs(pitchAngleDegrees) <= 
                   Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = false;
        }
        
        // Control drive system automatically, 
        // driving in reverse direction of pitch/roll angle,
        // with a magnitude based upon the angle
        
        if ( autoBalanceXMode ) {
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
            xAxisRate = Math.sin(pitchAngleRadians) * -1;
        }
        if ( autoBalanceYMode ) {
            double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
            yAxisRate = Math.sin(rollAngleRadians) * -1;
        }
        System.out.println("X " + xAxisRate + " -- " + "Y" + yAxisRate);
//        myRobot.mecanumDrive_Cartesian(xAxisRate, yAxisRate, stick.getTwist(),0);
//        Timer.delay(0.005);		// wait for a motor update time
        return false;
    }
  }


