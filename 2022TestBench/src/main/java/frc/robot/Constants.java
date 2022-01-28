// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static double joystickDeadband=0.2;
    public final static class MotorControllers{
        public final static int Motor1=21;
        public final static int Motor2=22;
        public final static int Motor3=23;
        public final static int Motor4=24;
        public final static int Shooter1=26;
        public final static int Shooter2=25;
    }
    public final static class InputControllers{
        public final static int kXboxLeft=0;
        public final static int kXboxRight=1;
    }
    public final static class SmartDashboardLabels{
        public final static String M1Speed="Motor 1 Speed";
        public final static String M2Speed="Motor 2 Speed";
        public final static String M3Speed="Motor 3 Speed";
        public final static String M4Speed="Motor 4 Speed";
        
    }
    public final static class ButtonMaps {
        //X Box Y Button =4
        //X Box B Button =2
        //X Box A Button =1
        //X Box X Button =3
        //X Box Left Bumper =5
        //X Box Right Bumper = 6

        public final static int SM1ForwardHighSpeed=4;
        public final static int SM1ForwardLowSpeed=99;
        public final static int SM1ReverseHighSpeed=4;
        public final static int SM1ReverseLowSpeed=99;

        public final static int SM2ForwardHighSpeed=2;
        public final static int SM2ForwardLowSpeed=99;
        public final static int SM2ReverseHighSpeed=2;
        public final static int SM2ReverseLowSpeed=99;
        
        public final static int SM3ForwardHighSpeed=1;
        public final static int SM3ForwardLowSpeed=99;
        public final static int SM3ReverseHighSpeed=1;
        public final static int SM3ReverseLowSpeed=99;

        public final static int SM4ForwardHighSpeed=3;
        public final static int SM4ForwardLowSpeed=99;
        public final static int SM4ReverseHighSpeed=3;
        public final static int SM4ReverseLowSpeed=99;

    }
}
