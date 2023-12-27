package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDSteering;
import frc.robot.classes.LEDSChargedup;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class zPipelaneSwapCommand extends CommandBase {
  private Limelight m_limeLight;
  private int m_pipeline = 0;
  private double m_targetHeight;
  private boolean m_waitingForpipelinereset = false;
  private boolean m_waitForPipeline;
  private double m_pipelineResetTime = 0;
  private final double kPipelineResetDelay = 0.8;

  private static LEDSChargedup fakeLeds = new LEDSChargedup();
  private LEDSChargedup useableLeds;

  public zPipelaneSwapCommand(Limelight limelight, int gridChoice, boolean waitforPipepline, LEDSChargedup ledStrip) {

    m_limeLight = limelight;
    m_waitForPipeline = waitforPipepline;
    // GRID Choice are 1 to 9 for the three rows in a grid
    // GRID 10 (top cone any column) 11 (middle cone any column) 12 (botton cone any
    // column)
    // GRID 13 (top cube column) 14 (middle cube any column), 15 (bottom cube any
    // column)
    switch (gridChoice) {
      // fall thorugh cases for same pipelines and heights
      case Constants.ChargedUp.GridPosUpperLeft:
      case Constants.ChargedUp.GridPosUpperRight:
      case Constants.ChargedUp.GridPosUpperConeAny:
        m_pipeline = Constants.VisionPipelines.TopTape;
        m_targetHeight = Constants.ChargedUp.targetHeightHighTape;
        break;
      case Constants.ChargedUp.GridPosMiddleLeft:
      case Constants.ChargedUp.GridPosMiddleRight:
      case Constants.ChargedUp.GridPosMiddleConeAny:
      case Constants.ChargedUp.GridPosBottomLeft:
      case Constants.ChargedUp.GridPosBottomRight:
      case Constants.ChargedUp.GridPosBottomConeAny:
        m_pipeline = Constants.VisionPipelines.BottomTape;
        m_targetHeight = Constants.ChargedUp.targetHeighMLowTape;
        break;
      case Constants.ChargedUp.playerStation:
        // will need to be 15 inches to the left of AprilTag
        m_pipeline = Constants.VisionPipelines.PlayerStationTag;
        m_targetHeight = Constants.ChargedUp.targetHeightPlayerStationTag;
        break;
      default:
        // Assume AprilTag if not a Cone Position
        m_pipeline = Constants.VisionPipelines.AprilTag;
        m_targetHeight = Constants.ChargedUp.targetHeightAprilTag;
    }
  }

  public zPipelaneSwapCommand(Limelight limelight, int gridChoice) {
    // constructor to support previous calls - assumes we will wait for pipeline to
    // be swapped
    this(limelight, gridChoice, true, fakeLeds);
  }

  public zPipelaneSwapCommand(Limelight limelight, int gridChoice, LEDSChargedup ledStrip) {
    this(limelight, gridChoice, true, ledStrip);

    // constructor to support previous calls - assumes we will wait for pipeline to
    // be swapped
  }

  @Override
  public void initialize() {
    //useableLeds.setPipelineLED(m_pipeline);
    if (m_limeLight.getPipeline() != m_pipeline) {
      System.out.println("Switching Pipeline - " + m_pipeline + " from " + m_limeLight.getPipeline());
      m_limeLight.setPipeline(m_pipeline);
      m_limeLight.setTargetHeight(m_targetHeight);
      m_limeLight.update();
      m_waitingForpipelinereset = true;
      m_pipelineResetTime = Timer.getFPGATimestamp();

    }
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    if (m_waitForPipeline = false) {
      return true; // if not waiting for pipeline to validate then end command
    } else {
      if (m_waitingForpipelinereset) {
        if (Timer.getFPGATimestamp() > m_pipelineResetTime + kPipelineResetDelay) {
          return true; // delay is over and pieline should be available
        } else {
          m_limeLight.update();
          return false;
        }
      } else {
        return true;
      }
    }
  }
}
