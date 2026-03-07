package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSys;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.subsystems.SwerveRotation;
import frc.robot.subsystems.AgitatorSys;
import frc.robot.subsystems.ShooterSys;
import frc.robot.subsystems.IntakeSys;

import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.LockCmd;
import frc.robot.commands.drivetrain.PointCmd;
import frc.robot.commands.drivetrain.AimToHubCmd;
import frc.robot.commands.functions.AgitatorCmd;
import frc.robot.commands.functions.AutoAimCmd;
import frc.robot.commands.functions.AutoShootCmd;
import frc.robot.commands.functions.IntakeCmd;
import frc.robot.commands.functions.IntakeStopCmd;
import frc.robot.commands.functions.RunShooterFFCmd;
import frc.robot.commands.functions.AutoAgitatorCmd;

public class RobotContainer {
    
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();
    private final SwerveRotation swerveRotation = new SwerveRotation(swerveSys);
    private final ShooterSys shooterSys = new ShooterSys(swerveSys);
    private final IntakeSys intakeSys = new IntakeSys();
    private final AgitatorSys agitatorSys = new AgitatorSys();

    //Initialize joysticks.
    public final static CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
    public final static CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorGamepadPort);

    //Name Commands
    PointCmd pointCmd;
    AutoShootCmd testCmd;
    AutoAimCmd autoPointCmd;
    RunShooterFFCmd runShooterFFCmd;
    IntakeCmd intakeCmd;
    AgitatorCmd agitatorCmd;
    AimToHubCmd aimToHubCmd;
    IntakeStopCmd intakeStopCmd;
    AutoAgitatorCmd autoAgitatorCmd;

    //Initialize auto selector.
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    //private UsbCamera camera;

    public RobotContainer() {
        RobotController.setBrownoutVoltage(DriveConstants.brownoutVoltage);

        // Register Commands to PathPlanner
        NamedCommands.registerCommand("Aim", new AutoAimCmd(swerveSys));
        NamedCommands.registerCommand("Shoot", new AutoShootCmd(shooterSys));
        NamedCommands.registerCommand("Agitate", new AutoAgitatorCmd(agitatorSys));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoSelector = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("auto select", autoSelector);

    //Initalize Commands
        pointCmd = new PointCmd(swerveRotation);
        testCmd = new AutoShootCmd(shooterSys);
        autoPointCmd = new AutoAimCmd(swerveSys);
        runShooterFFCmd = new RunShooterFFCmd(shooterSys);
        intakeCmd = new IntakeCmd(intakeSys, false);
        agitatorCmd = new AgitatorCmd(agitatorSys, false);
        aimToHubCmd = new AimToHubCmd(swerveSys);
        intakeStopCmd = new IntakeStopCmd(intakeSys);
        autoAgitatorCmd = new AutoAgitatorCmd(agitatorSys);

        //Add Requirements
    // pointCmd already requires the lightweight rotation subsystem. No need to add SwerveSys requirement.
            

        new EventTrigger("Intake2").onTrue(new IntakeCmd(intakeSys, false));
        new EventTrigger("Intake2").onFalse(new IntakeStopCmd(intakeSys));


        configDriverBindings();
        configOperatorBindings();

    }

    private void configOperatorBindings() {
    operatorController.b().whileTrue(new AgitatorCmd(agitatorSys, false));
    operatorController.a().whileTrue(new AgitatorCmd(agitatorSys, true));
    operatorController.leftTrigger().whileTrue(new IntakeCmd(intakeSys, false));
    operatorController.leftBumper().whileTrue(new IntakeCmd(intakeSys, true));
    operatorController.rightTrigger().whileTrue(new RunShooterFFCmd(shooterSys));
    }

    public void configDriverBindings() {
        swerveSys.setDefaultCommand(new ArcadeDriveCmd(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
            true,
            true,
            swerveSys));

        driverController.start().onTrue(Commands.runOnce(() -> swerveSys.resetHeading()));

        //Swerve locking system
        driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
           .whileTrue(new LockCmd(swerveSys));

        driverController.rightTrigger().whileTrue(aimToHubCmd);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    } 

    // For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
    public void updateInterface() {
        
        SmartDashboard.putNumber("heading degrees", swerveSys.getHeading().getDegrees());
        SmartDashboard.putNumber("speed m/s", swerveSys.getAverageDriveVelocityMetersPerSec());

        SmartDashboard.putNumber("pose x meters", swerveSys.getPose().getX());
        SmartDashboard.putNumber("pose y meters", swerveSys.getPose().getY());

        SmartDashboard.putNumber("blue pose x meters", swerveSys.getBlueSidePose().getX());

        SmartDashboard.putNumber("FL angle degrees", swerveSys.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("FR angle degrees", swerveSys.getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("BL angle degrees", swerveSys.getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("BR angle degrees", swerveSys.getModuleStates()[3].angle.getDegrees());

        SmartDashboard.putNumber("FL raw CANCoder degrees", swerveSys.getCanCoderAngles()[0].getDegrees());
        SmartDashboard.putNumber("FR raw CANCoder degrees", swerveSys.getCanCoderAngles()[1].getDegrees());
        SmartDashboard.putNumber("BL raw CANCoder degrees", swerveSys.getCanCoderAngles()[2].getDegrees());
        SmartDashboard.putNumber("BR raw CANCoder degrees", swerveSys.getCanCoderAngles()[3].getDegrees());

        SmartDashboard.putNumber("FL offset CANCoder degrees", swerveSys.getCanCoderAngles()[0].getDegrees() - DriveConstants.frontLeftModOffset.getDegrees());
        SmartDashboard.putNumber("FR offset CANCoder degrees", swerveSys.getCanCoderAngles()[1].getDegrees() - DriveConstants.frontRightModOffset.getDegrees());
        SmartDashboard.putNumber("BL offset CANCoder degrees", swerveSys.getCanCoderAngles()[2].getDegrees() - DriveConstants.backLeftModOffset.getDegrees());
        SmartDashboard.putNumber("BR offset CANCoder degrees", swerveSys.getCanCoderAngles()[3].getDegrees() - DriveConstants.backRightModOffset.getDegrees());

        SmartDashboard.putNumber("drive voltage", swerveSys.getAverageDriveVoltage());

        SmartDashboard.putNumber("Shooter RPM", shooterSys.getShooterRPM());
        SmartDashboard.putNumber("Desired Shooter RPM", shooterSys.desiredRPM());
        SmartDashboard.putNumber("Limelight TY", LimelightHelpers.getTY(VisionConstants.LimelightName));

        SmartDashboard.putNumberArray("Pose in Target Space", LimelightHelpers.getBotPose_TargetSpace(VisionConstants.LimelightName));
        SmartDashboard.putNumber("IntakeAmps", intakeSys.getIntakeAmps());
        SmartDashboard.putNumber("IntakeTemp", intakeSys.getIntakeTemp());

    }   
}
