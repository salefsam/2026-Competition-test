package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.FieldConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;


public class ShooterSys extends SubsystemBase {

    SparkFlex shooterMtr;
    RelativeEncoder shooterEnc;
    SparkClosedLoopController shooterController;
    SwerveSys swerveSys;

    public ShooterSys(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        
        shooterMtr = new SparkFlex(CANDevices.shooterMtrId, MotorType.kBrushless);
        shooterEnc = shooterMtr.getEncoder();
        shooterController = shooterMtr.getClosedLoopController();

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.stallLimitAmps, ShooterConstants.freeLimitAmps, ShooterConstants.maxRPM);
        shooterConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1)
            .uvwMeasurementPeriod(15);
        shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ShooterConstants.shooterkP, ShooterConstants.shooterkI, ShooterConstants.shooterkD);

        shooterMtr.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

     // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_angle = Radians.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);


    private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motor(s).
            (voltage) -> setShooterVolts(voltage),
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shooterMtr.get() * 12, Volts))
                    .angularPosition(m_angle.mut_replace(shooterEnc.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(getShooterRPM(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));


    public void setShooterRPM(double rpm) {
        shooterController.setSetpoint(rpm, ControlType.kVelocity);
    }
    
    public void setShooterVolts(Voltage volts) {
        shooterMtr.setVoltage(volts);
    }

    public void setControllerVolts(double volts){
        shooterController.setSetpoint(volts, ControlType.kVoltage);
    }

    public double getShooterRPM() {
        return shooterEnc.getVelocity();
    }

    public void stop() {
        shooterController.setSetpoint(0, ControlType.kVelocity);
    }

        /**
     * Returns the planar (ground) distance from the shooter's exit point to the center
     * of the alliance hub in feet. Uses Limelight field pose (when valid) and falls back
     * to odometry (`swerveSys.getPose()`) otherwise.
     *
     * This handles the case where the robot isn't square to the hub by computing the
     * true Euclidean distance between the shooter's position and the hub center.
     */
    public double getDistanceCenterHub() {
        double meters = getPlanarDistanceToHubMeters();
        return Units.metersToFeet(meters);
    }

    /**
     * Compute the planar (XY) distance in meters between the shooter's exit point and the
     * hub center. This uses the Limelight's field pose if available and meets the
     * target-area threshold; otherwise it uses the drivetrain odometry pose.
     */
    public double getPlanarDistanceToHubMeters() {
        // Choose hub position by alliance
        edu.wpi.first.wpilibj.DriverStation.Alliance alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()
            ? edu.wpi.first.wpilibj.DriverStation.getAlliance().get()
            : edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

        Translation2d hubTranslation = (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red)
            ? FieldConstants.redAllianceHubPose
            : FieldConstants.blueAllianceHubPose;

        // Prefer Limelight pose when it is reporting a visible target with sufficient area.
        Pose2d robotPose2d;
        boolean limelightHasTarget = LimelightHelpers.getTV(VisionConstants.LimelightName) &&
            LimelightHelpers.getTA(VisionConstants.LimelightName) >= VisionConstants.targetAreaPercentThreshold;

        if (limelightHasTarget) {
            // Use the alliance-specific wpi pose the Limelight publishes
            if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                robotPose2d = LimelightHelpers.getBotPose2d_wpiRed(VisionConstants.LimelightName);
            } else {
                robotPose2d = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.LimelightName);
            }
            // If the limelight pose looks like an all-zero default, fall back to odometry
            if (robotPose2d.getTranslation().getX() == 0.0 && robotPose2d.getTranslation().getY() == 0.0) {
                robotPose2d = swerveSys.getPose();
            }
        } else {
            robotPose2d = swerveSys.getPose();
        }

        // Apply shooter offset (robot -> shooter) using the rotation of the robot so the offset
        // is in field coordinates.
        Transform2d shooterOffsetTransform = new Transform2d(
            new Translation2d(ShooterConstants.shooterOffsetXMeters, ShooterConstants.shooterOffsetYMeters),
            new Rotation2d(0.0));

        Pose2d shooterPose = robotPose2d.transformBy(shooterOffsetTransform);

        double dx = shooterPose.getTranslation().getX() - hubTranslation.getX();
        double dy = shooterPose.getTranslation().getY() - hubTranslation.getY();

        return Math.hypot(dx, dy);
    }

    public double desiredRPM() {

        return 3900;

    }

    @Override
    public void periodic() {

    }
    /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

    
}
