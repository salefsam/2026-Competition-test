package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.*;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    //region START
    @Override
    public void robotInit() {
        //Runs one at program start
        //Initialize RobotContainer - setup button bindings, subsystems, and commands
        robotContainer = new RobotContainer();
        RobotController.setBrownoutVoltage(RobotConstants.brownoutVoltage);
    }
    //endregion


    //region RUN
    @Override
    public void robotPeriodic() {
        //Runs every ~20ms
        CommandScheduler.getInstance().run();
        robotContainer.updateDashboard();
    }
    //endregion



    //region AUTONOMOUS STATE
    @Override
    public void autonomousInit() {
        //Runs once when autonomous starts
        autonomousCommand = robotContainer.getAutonomousCommand();

        if(autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
        //runs every loop in auto
        //Should not really be needed
    }
    //endregion



    //region TELEOP STATE
    @Override
    public void teleopInit() {
        //Runs once when teleop starts
        if(autonomousCommand != null) autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {
        //runs every loop in teleop
        //Should not need this if we're command based
    }
    //endregion



    //region TEST STATE
   @Override
    public void testInit() {
        //runs once when test mode starts
    }

    @Override
    public void testPeriodic() {
        //runs every loop in test mode
    }
    //endregion



    //region DISABLED STATE
    @Override
    public void disabledInit() {
        //runs once when disabled - should stop motors, reset sensors, etc.
    }

    @Override
    public void disabledPeriodic() {
        //runs every loop when disabled
        /*Could be helpful to put in 
        robotContainer.updateInterface();
        * If we wanted to get telemetry or sensor output when we're disabled
        */
    }
    //endregion

}
