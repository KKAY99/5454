package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class ClawOpenCloseCommand extends CommandBase {

    private PneumaticsSubsystem m_pneumatics;

    public ClawOpenCloseCommand(PneumaticsSubsystem pneumatics) {
        m_pneumatics = pneumatics;
    }

    @Override
    public void execute() {
        if (m_pneumatics.getTopClawSolenoidState() == true) {
            m_pneumatics.setTopClawSolenoid(false);
            m_pneumatics.setBottomClawSolenoid(false);
        } else {
            m_pneumatics.setTopClawSolenoid(true);
            m_pneumatics.setBottomClawSolenoid(true);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
