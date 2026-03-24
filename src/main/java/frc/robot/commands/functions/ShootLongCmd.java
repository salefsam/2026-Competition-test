package frc.robot.commands.functions;

import frc.robot.subsystems.ShooterSys;
import frc.robot.Constants.*;

public class ShootLongCmd extends AutoShootCmd {
    private static final double DURATION = AutoConstants.LONG_DURATION_SHOOT_TIME;

    public ShootLongCmd(ShooterSys shooter) {
        super(shooter, DURATION);
    }
}
