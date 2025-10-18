 package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public final class Constants {
    public static final int p = 0;
    public static final int i = 0;
    public static final int d = 0;
    public static final int k30Amp = 30;
    public static final int k10Amp = 10;
    public static final int k5Amp = 5;
    public static final int k2Amp = 2;
    public static final double intakeInSpeed=-0.3;
    public static final double intakeOutSpeed=1;
    public static final double climbSpeed=-0.6;
    public static final double climbDownSpeed=0.6;
    public static final double rotateUpSpeed=0.3;
    public static final double rotateDownSpeed=-0.6;

    public static final class AutomationConstants{
        public static final double autoRotateSpeed=0.3;
        public static final double autoIntakeSpeed=.6;
        public static final double autoOuttakeSpeed=-1;
        public static final double autoScoreSpeed=-1;
        public static final double autoIntakeTargetPos=40;
        public static final double autoOutakeTargetPos=-0;
        public static final double autoDeadband=1;
    }
    public static final class ButtonConstants{
        public static final int ClimbUp=4;
        public static final int DriverIntakeIn=3;
        public static final int DriverOutake=2;
        public static final int RotateUp=8;
        public static final int RotateDown=1;
        public static final int AutoIntake=5;
        public static final int AutoOutake=6;
        public static final int POVClimbUp=0;
        public static final int POVClimbDown=180;
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
