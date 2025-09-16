package frc.robot;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.Constants.InputControllers;


public class RobotContainer {
  private final Field2d m_Field2d = new Field2d();
  
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();
public class RobotContainer {
    
    // The robot's subsystems and commands are defined here...
    private ClimbSubsystem m_Climb = new ClimbSubsystem(61);
    private IntakeSubsystem m_Intake =  new IntakeSubsystem(60, 62);
    
    //public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

    // Dashboard inputs
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
  
    private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    //private final SendableChooser<Command> m_autoChooser;
    public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

    private final SendableChooser<Boolean> m_IsDrone = new SendableChooser<>();
    // private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem(Constants.Spindexer.motorPort);
 
 // reversed 9/21

    
    public RobotContainer() {
        //m_autoChooser = AutoBuilder.buildAutoChooser();
        createAutoCommandsList();
        configureNamedCommands();
        configureButtonBindings();
        resetDefaultCommand();
    }

  public boolean m_hasResetGyro=false;

  
  public RobotContainer(){
      
    configureButtonBindings();
    resetDefaultCommand();
  }



  private void configureButtonBindings(){
  
    GasPedalCommand gasPedalCommand=new GasPedalCommand(m_swerve,()->m_xBoxDriver.getRightTriggerAxis());
    m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);

  }


  public void AutonMode(){
  }

  public void TeleopMode(){
    
  }
  
  public void TeleopPeriodic(){
    
  }

  public void AllPeriodic(){
    m_Field2d.setRobotPose(m_swerve.getPose2d());
  }

  }
}


  
  