package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class HoldArmSwapCommand extends CommandBase {

    private PneumaticsSubsystem m_pneumatics;

    public HoldArmSwapCommand(PneumaticsSubsystem pneumatics) {
        m_pneumatics = pneumatics;
    }

    @Override
    public void execute() {
        if (m_pneumatics.getArmHoldCylender()) {
            System.out.println("manual arm hold false");
            m_pneumatics.setArmHoldCynlinder(false);
        } else {
            System.out.println("manual arm hold true");
            m_pneumatics.setArmHoldCynlinder(true);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
