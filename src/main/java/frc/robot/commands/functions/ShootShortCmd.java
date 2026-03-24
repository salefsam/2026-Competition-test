package frc.robot.commands.functions;

import frc.robot.subsystems.ShooterSys;
import frc.robot.Constants.*;

public class ShootShortCmd extends AutoShootCmd {
    private static final double DURATION = AutoConstants.SHORT_DURATION_SHOOT_TIME;

    public ShootShortCmd(ShooterSys shooter) {
        super(shooter, DURATION);
    }
}
