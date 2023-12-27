package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;

public class PipelineSwapCommand extends CommandBase{
    private Limelight m_limeLight;
    private int m_pipeline;
    private double m_targetHeight;

    public PipelineSwapCommand(Limelight limeLight, int pipeline, double targetHeight){
        m_pipeline = pipeline;
        m_limeLight = limeLight;
        m_targetHeight = targetHeight;
    }

    @Override
    public void execute(){
        if(m_limeLight.getPipeline()!=m_pipeline){
            //System.out.println("Switching Pipeline - " + m_pipeline + " from " + m_limeLight.getPipeline());
              m_limeLight.setPipeline(m_pipeline);
              m_limeLight.setTargetHeight(m_targetHeight);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
