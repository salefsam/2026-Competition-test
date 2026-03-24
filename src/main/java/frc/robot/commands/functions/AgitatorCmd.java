package frc.robot.commands.functions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSys;
import frc.robot.Constants.AutoConstants;

public class AgitatorCmd extends Command {

    public enum AgitatorMode { FORWARD, REVERSE, STOP, AUTO }

    private final AgitatorSys agitatorSys;
    private final AgitatorMode mode;
    private final double duration; // for AUTO mode
    private final Timer timer = new Timer();
    private static final double START_DELAY = 0.5; // seconds
    private boolean started = false;

    // Constructor for manual direction commands (FORWARD, REVERSE, STOP)
    public AgitatorCmd(AgitatorSys agitatorSys, AgitatorMode mode) {
        this.agitatorSys = agitatorSys;
        this.mode = mode;
        this.duration = 0;
        addRequirements(agitatorSys);
    }

    // Constructor for timed AUTO mode
    public AgitatorCmd(AgitatorSys agitatorSys, double duration) {
        this.agitatorSys = agitatorSys;
        this.mode = AgitatorMode.AUTO;
        this.duration = duration;
        addRequirements(agitatorSys);
    }

    @Override
    public void initialize() {
        if (mode == AgitatorMode.AUTO) {
            timer.reset();
            timer.start();
            started = false;
        }
    }

    @Override
    public void execute() {
        switch (mode) {
            case FORWARD:
                agitatorSys.setAgitatorRPM(false); // forward
                break;
            case REVERSE:
                agitatorSys.setAgitatorRPM(true); // reverse
                break;
            case STOP:
                agitatorSys.stop();
                break;
            case AUTO:
                if (timer.hasElapsed(START_DELAY)) {
                    agitatorSys.setAgitatorRPM(false); // forward during AUTO
                    started = true;
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        agitatorSys.stop();
        if (mode == AgitatorMode.AUTO) timer.stop();
    }

    @Override
    public boolean isFinished() {
        if (mode == AgitatorMode.AUTO) {
            return timer.hasElapsed(START_DELAY + duration);
        }
        return false; // manual commands never finish automatically
    }

    // Convenience static inner classes
    public static class AgitateForwardCmd extends AgitatorCmd {
        public AgitateForwardCmd(AgitatorSys agitatorSys) {
            super(agitatorSys, AgitatorMode.FORWARD);
        }
    }

    public static class AgitateReverseCmd extends AgitatorCmd {
        public AgitateReverseCmd(AgitatorSys agitatorSys) {
            super(agitatorSys, AgitatorMode.REVERSE);
        }
    }

    public static class AgitateStopCmd extends AgitatorCmd {
        public AgitateStopCmd(AgitatorSys agitatorSys) {
            super(agitatorSys, AgitatorMode.STOP);
        }
    }

    public static class AgitateShortCmd extends AgitatorCmd {
        public AgitateShortCmd(AgitatorSys agitatorSys) {
            super(agitatorSys, AutoConstants.SHORT_DURATION_AGITATE_TIME);
        }
    }

    public static class AgitateLongCmd extends AgitatorCmd {
        public AgitateLongCmd(AgitatorSys agitatorSys) {
            super(agitatorSys, AutoConstants.LONG_DURATION_AGITATE_TIME);
        }
    }
}