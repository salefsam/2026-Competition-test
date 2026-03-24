package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSys;

public class IntakeCommands extends Command {

    public enum IntakeDirection { IN(0.8), OUT(-0.8), STOP(0.0); 
        public final double speed;
        IntakeDirection(double speed) { this.speed = speed; }
    }

    private final IntakeSys intakeSys;
    private final IntakeDirection direction;

    public IntakeCommands(IntakeSys intakeSys, IntakeDirection direction) {
        this.intakeSys = intakeSys;
        this.direction = direction;
        addRequirements(intakeSys);
    }

    @Override
    public void execute() {
        intakeSys.setRollerRPM(direction.speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSys.stop();
    }

    @Override
    public boolean isFinished() {
        return false;  // Continuous command until released
    }

    // Optional convenience subclasses
    public static class IntakeInCmd extends IntakeCommands {
        public IntakeInCmd(IntakeSys intakeSys) {
            super(intakeSys, IntakeDirection.IN);
        }
    }

    public static class IntakeOutCmd extends IntakeCommands {
        public IntakeOutCmd(IntakeSys intakeSys) {
            super(intakeSys, IntakeDirection.OUT);
        }
    }

    public static class IntakeStopCmd extends IntakeCommands {
        public IntakeStopCmd(IntakeSys intakeSys) {
            super(intakeSys, IntakeDirection.STOP);
        }
    }
}