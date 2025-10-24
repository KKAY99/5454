package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkinDonutSubsystem;

public class CoralShootCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  
    DunkinDonutSubsystem m_dunkin;
    double m_speed;
    public CoralShootCommand(DunkinDonutSubsystem dunkin,double speed){
        m_dunkin=dunkin;
        m_speed=speed;
        addRequirements(dunkin);

    }
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        m_dunkin.runCoralShootMotor(m_speed);
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        m_dunkin.stopCoralShootMotor();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
}
