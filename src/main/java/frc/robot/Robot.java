package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;


public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        final RobotContainer robotContainer = new RobotContainer();

        Logger.configureLoggingAndConfig(robotContainer, false);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Logger.updateEntries();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }


    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

}
