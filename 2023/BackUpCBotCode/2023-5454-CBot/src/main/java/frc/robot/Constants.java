// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int gyroPort=0;

public class Arm {
  public static final double homePos=1;
  public static final double shootPos1=0.944;
  public static final double shootPos2=0.7816;
  public static final double shootPos3=0.576;
  public static final int motorPort=32;
  public static final int encoderPort=0;
  public static final double homePost=0;
  public static final double fastSpeed=0.7;
  public static final double slowSpeed=0.4;
  public static final double manualSpeed=0.5;
  public static final double minValue=0.45;
  public static final double maxValue=0.95;
}
public class ShooterSubsystem{
  public static final double snowMotorSpeed=1.0;
  public static final double shootLowSpeed=0.2;
  public static final double shootMediumSpeed=0.3;
  public static final double shootHighSpeed=0.5;
  public static final double delayLowShot=0.2;
  public static final double delayMediumShot=0.7;
  public static final double delayHighShot=1.0;
  public static final double shootTime=1;
  public static final double intakeSpeed=0.7;
  public static final int leftShootPort=21;
  public static final int rightShootPort=22;
  public static final int snowMotorPort=11;
  public static final int limitSwitch=1;
}

  public class buttonConstants {
    public static final double DeadBand=0.06;
    public static final int moveArmUp=4;
    public static final int moveArmDown=1;
    public static final int shootPos1Button=2;
    public static final int shootPos2Button=3;
    public static final int shootPos3Button=5;
    public static final int homePosButton=10;
    public static final int intakeCube=1;
    public static final int shootCubeHigh=2;
    public static final int shootCubeMedium=3;
    public static final int shootCubeLow=4;
  }



    public static final class InputControllers{
      public static final int kXboxDriver=0;
      public static final int kXboxOperator=1;
}



    public class zHomeArmCommand {
      public static final double rotateSpeed=0.1;
    }
    public static final class AutoModes {
      public static final String autoMode0="Do Nothing";
      public static final String autoMode1="Move Foward";
      public static final String autoMode2="Score and Move Foward";
      public static final String autoMode3="Score and Balance";
    
      public static final int autoNothing = 0;
      public static final int autoMoveForward = 1;
      public static final int autoMoveScoreMoveForward = 2;
      public static final int autoMoveScoreBalance = 3;
      
      public static final String delayMode0="0 Seconds";
      public static final String delayMode1="3 Seconds";
      public static final String delayMode2="5 Seconds";
      public static final String delayMode3="8 Seconds";
      public static final int delayValMode0=0;
      public static final int delayValMode1=3;
      public static final int delayValMode2=5;
      public static final int delayValMode3=8;
      public static final int defaultDelayMode=0;
      
      public static final double MoveSpeed=0.2;
      public static final double MoveOutDistance=10;
      public static final double EngageDistance=5;


  }
}
