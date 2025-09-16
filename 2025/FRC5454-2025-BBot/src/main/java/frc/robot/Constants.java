 package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public final class Constants {
    public static final int p = 0;
    public static final int i = 0;
    public static final int d = 0;
    public static final int k30Amp = 30;
 
    public static final class ButtonConstants{
        public static final int DriverIntakeIn=1;
    }

    public static final class InputControllers {
        public static final int kXboxDrive = 0;
        public static final int kXboxOperator = 1;
        public static final int kCustomController = 2;
    }

    public static final class DriveConstants{
        // Joystick axis deadband for the swerve drive
        public static final double swerveDeadband = 0.1;
        //Rotate Joystick axis deadband - bigger deadband to avoid rotational drift
        public static final double swerveRotateDeadband = 0.12; //0.17
        public static final double MinGasPedalSpeed=0.20;
      }


    public static final PPHolonomicDriveController pathPlanDriveController = new PPHolonomicDriveController(
        new PIDConstants(5.0, 0, 0), // Translation constants 
        new PIDConstants(25.0, 0, 1) // Rotation constants
      );
}
