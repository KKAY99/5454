package frc.robot.commands;
import frc.robot.subsystems.PnuematicsSubystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Claw;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ClawIntakeCommand extends CommandBase {
    private PnuematicsSubystem m_Pnuematics;
    private ClawSubsystem m_Claw;

    private static final double kClawRunTime=1;
    private double m_endTime;
    private boolean isRunning=false;
    private DoubleSupplier m_speed;
    
    public ClawIntakeCommand(ClawSubsystem Claw, DoubleSupplier speed){
        m_Claw=Claw;
        m_speed=speed;
    }

    @Override
    public void execute() {
        m_Claw.runClaw(m_speed.getAsDouble());
    }
  
    @Override
    public void end(boolean interrupted) {
        m_Claw.stopClaw();
    }
  
    @Override
    public boolean isFinished() {
        return false;
    }

}