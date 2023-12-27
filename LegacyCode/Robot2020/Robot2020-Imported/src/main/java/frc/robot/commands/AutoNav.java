package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class AutoNav extends CommandBase {
    private DriveSubsystem m_drive;
    public AutoNav(DriveSubsystem driveSubsystem,int navCourse) {
        m_drive=driveSubsystem;
        driveSquare();
}
    private void driveSquare(){
        
    
        m_drive.commandDriveStraight(.20,3);
        Timer.delay(3);
        m_drive.commandTurnRobot(.30, 65 );
        Timer.delay(3);
        m_drive.commandDriveStraight(.20,3);
        Timer.delay(3);
        m_drive.commandTurnRobot(.30, 65 );
        Timer.delay(3);
        m_drive.commandDriveStraight(.20,3);
        Timer.delay(3);
        m_drive.commandTurnRobot(.30, 65 );
        Timer.delay(3);
        m_drive.commandDriveStraight(.20,3);
    } 
}
