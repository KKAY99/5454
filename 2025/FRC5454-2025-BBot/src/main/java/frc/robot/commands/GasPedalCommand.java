package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GasPedalCommand extends Command{
    private CommandSwerveDrivetrain m_swerve;

    private DoubleSupplier m_speed;

    public GasPedalCommand(CommandSwerveDrivetrain swerve,DoubleSupplier speed){
        m_swerve=swerve;
        m_speed=speed;
    }

    @Override
    public void end(boolean interrupted){
        m_swerve.setGasPedalMult(1,1);
    }

    @Override
    public boolean isFinished(){
        double gasPedalValue=MathUtil.clamp(Math.abs(m_speed.getAsDouble()-1),0.1,0.5);
        
        m_swerve.setGasPedalMult(gasPedalValue,0.8);
        return false;
    }
}
