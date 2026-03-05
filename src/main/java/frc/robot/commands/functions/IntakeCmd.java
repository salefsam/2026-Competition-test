package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSys;
import frc.robot.Constants.RollerConstants;

public class IntakeCmd extends Command {

    private final IntakeSys intakeSys;
    //private final int direction;
    
     public IntakeCmd(IntakeSys intakeSys) {
         this.intakeSys = intakeSys;
    }

    //  public IntakeCmd(IntakeSys intakeSys, int direction) {
    //      this.intakeSys = intakeSys;
    //      this.direction = direction;
    //  }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        // if (direction == 1){
        //     intakeSys.setRollerRPM(RollerConstants.rollerRPM*direction);    
        // }
        // else {
        //     intakeSys.setRollerRPM(RollerConstants.rollerRPM*direction);
        // }
        
        //In theory we can just use this:
        //intakeSys.setRollerRPM(RollerConstants.rollerRPM * direction);
        intakeSys.setRollerRPM(RollerConstants.rollerRPM);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSys.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}