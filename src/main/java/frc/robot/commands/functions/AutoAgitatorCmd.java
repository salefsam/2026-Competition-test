package frc.robot.commands.functions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSys;

public class AutoAgitatorCmd extends Command {

    private final AgitatorSys agitatorSys;
    private final Timer timer;
    private final double duration = 10.0; // seconds

    public AutoAgitatorCmd(AgitatorSys agitatorSys) {
        this.agitatorSys = agitatorSys;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        agitatorSys.setAgitatorRPM(false);
    }

    @Override
    public void end(boolean interrupted) {
        agitatorSys.stop();
        timer.stop();
    }
    
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
    
}
