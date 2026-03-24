package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.functions.*;
import frc.robot.commands.functions.AgitatorCmd.AgitateForwardCmd;
import frc.robot.commands.functions.AgitatorCmd.AgitateLongCmd;
import frc.robot.commands.functions.AgitatorCmd.AgitateReverseCmd;
import frc.robot.commands.functions.AgitatorCmd.AgitateShortCmd;
import frc.robot.commands.functions.AgitatorCmd.AgitateStopCmd;
import frc.robot.commands.functions.IntakeCommands.IntakeInCmd;
import frc.robot.commands.functions.IntakeCommands.IntakeOutCmd;
import frc.robot.commands.functions.IntakeCommands.IntakeStopCmd;
//import frc.robot.commands.lights.*;
//import frc.robot.util.*;
//import frc.robot.util.led.*;
import frc.robot.util.limelight.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

//This is the brains of our robot
//We should Instantiate subsystems, set default commands, bind buttons, and provide autonomous commands
//We should not put logic or motor controls in here
public class RobotContainer {
    
    // Instantiate subsystems
    //drivetrain - Can we combine SwerveSys with SwerveRotation?
    private final SwerveSys swerveSys = new SwerveSys();
    private final SwerveRotation swerveRotation = new SwerveRotation(swerveSys);
    private final ShooterSys shooterSys = new ShooterSys(swerveSys);
    private final IntakeSys intakeSys = new IntakeSys();
    private final AgitatorSys agitatorSys = new AgitatorSys();

    //Named / Reusable Commands
    private AutoShootCmd testCmd;
    private PointCmd pointCmd;
    private AutoAimCmd autoPointCmd;
    private AimToHubCmd aimToHubCmd;
    private RunShooterFFCmd runShooterFFCmd;
    private AgitatorCmd.AgitateForwardCmd agitateForwardCmd;
    private AgitatorCmd.AgitateReverseCmd agitateReverseCmd;
    private AgitatorCmd.AgitateLongCmd agitateLongCmd;
    private AgitatorCmd.AgitateShortCmd agitateShortCmd;
    private AgitatorCmd.AgitateStopCmd agitateStopCmd;
    private IntakeInCmd intakeInCmd;
    private IntakeOutCmd intakeOutCmd;
    private IntakeStopCmd intakeStopCmd;

    //Instantiate Controllers
    public final static CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
    public final static CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorGamepadPort);

    //PDH
    private final PowerDistribution pdh = new PowerDistribution(CANDevices.pdhId, ModuleType.kRev);

    private SendableChooser<Command> autoSelector;

    //Shuffleboard
    // in RobotContainer.java
    private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    private final ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");
    private final ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
    private final ShuffleboardTab systemTab = Shuffleboard.getTab("System");

    // Drivetrain layout
    private final ShuffleboardLayout drivetrainLayout = drivetrainTab.getLayout("Drivetrain", BuiltInLayouts.kGrid)
    .withSize(4, 2)
    .withPosition(0, 0);
    private final ShuffleboardLayout operatorLayout = operatorTab.getLayout("Operator", BuiltInLayouts.kGrid)
    .withSize(4,2)
    .withPosition(0,0);
    private final ShuffleboardLayout driverLayout = driverTab.getLayout("Driver", BuiltInLayouts.kGrid)
    .withSize(4,2)
    .withPosition(0, 0);
    private final ShuffleboardLayout systemLayout = systemTab.getLayout("System Health", BuiltInLayouts.kGrid)
    .withSize(4,2)
    .withPosition(0, 0);


    //Constructor
    public RobotContainer() {
        //initialize reusable commands first        
        //aim
        testCmd = new AutoShootCmd(shooterSys, 10);
        pointCmd = new PointCmd(swerveRotation);
        autoPointCmd = new AutoAimCmd(swerveSys);
        aimToHubCmd = new AimToHubCmd(swerveSys);
        //shoot
        runShooterFFCmd = new RunShooterFFCmd(shooterSys);
        //agitate
        agitateForwardCmd = new AgitateForwardCmd(agitatorSys);
        agitateReverseCmd = new AgitateReverseCmd(agitatorSys);
        agitateLongCmd = new AgitateLongCmd(agitatorSys);
        agitateShortCmd = new AgitateShortCmd(agitatorSys);
        agitateStopCmd = new AgitateStopCmd(agitatorSys);
        //intake
        intakeStopCmd = new IntakeStopCmd(intakeSys);
        

        //Register named commands to PathPlanner
        NamedCommands.registerCommand("Aim", new AutoAimCmd(swerveSys));
        NamedCommands.registerCommand("Shoot", new ShootLongCmd(shooterSys));
        NamedCommands.registerCommand("Shoot2Sec", new ShootShortCmd(shooterSys));
        NamedCommands.registerCommand("Agitate", new AgitateLongCmd(agitatorSys));
        NamedCommands.registerCommand("Agitate2Sec", new AgitateShortCmd(agitatorSys));

        //configure defaults
        configureDefaultCommands();
        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        //and add to dashboard
        autoSelector = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto select", autoSelector);
        
        //setup event triggers
        EventTrigger intake2 = new EventTrigger("Intake2");
        intake2.onTrue(new IntakeInCmd(intakeSys));
        intake2.onFalse(new IntakeStopCmd(intakeSys));
    }

    private void configureDefaultCommands() {
        swerveSys.setDefaultCommand(new ArcadeDriveCmd(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
            true,
            true,
            swerveSys));
        //agitator idle command
        //agitatorSys.setDefaultCommand(new AgitatorIdleCmd(agitatorSys));

        //shooter idle command
        //shooterSys.setDefaultCommand(new ShooterIdleCmd(shooterSys));
    }

    private void configureButtonBindings() {
        //Driver
        driverController.start().onTrue(Commands.runOnce(() -> swerveSys.resetHeading()));

        //Swerve locking system
        driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
           .whileTrue(new LockCmd(swerveSys));

        driverController.rightTrigger().whileTrue(aimToHubCmd);

        //Operator
        operatorController.b().whileTrue(new AgitatorCmd.AgitateReverseCmd(agitatorSys));
        operatorController.a().whileTrue(new AgitatorCmd.AgitateForwardCmd(agitatorSys));
        operatorController.leftTrigger().whileTrue(new IntakeInCmd(intakeSys));
        operatorController.leftBumper().whileTrue(new IntakeOutCmd(intakeSys));
        operatorController.rightTrigger().whileTrue(new RunShooterFFCmd(shooterSys));
    }

     public Command getAutonomousCommand() {
         //return new ExampleAutoCommand(drivetrain, shooter, agitator, intake);
         return autoSelector.getSelected();
     }

    // For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
    public void updateDashboard() {

        //Operator
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        operatorLayout.addNumber(null, null);
        SmartDashboard.putNumber("IntakeAmps", intakeSys.getIntakeAmps());
        SmartDashboard.putNumber("IntakeTemp", intakeSys.getIntakeTemp());
        SmartDashboard.putNumber("Shooter RPM", shooterSys.getShooterRPM());
        SmartDashboard.putNumber("Desired Shooter RPM", shooterSys.desiredRPM());
        SmartDashboard.putNumber("ShooterError", shooterSys.desiredRPM() - shooterSys.getShooterRPM());
        SmartDashboard.putNumber("Limelight TY", LimelightHelpers.getTY(VisionConstants.LimelightName));
        SmartDashboard.putNumber("DistanceToCenterHub", shooterSys.getPlanarDistanceToHubMeters());
        SmartDashboard.putBoolean("Is Aiming", aimToHubCmd.isScheduled());
        SmartDashboard.putBoolean("Vision Locked", LimelightHelpers.getTV(VisionConstants.LimelightName));
        SmartDashboard.putBoolean("Shooter Running", runShooterFFCmd.isScheduled());
        SmartDashboard.putBoolean("READY TO SHOOT", shooterSys.atSetpoint());

        //Driver
        driverLayout.addNumber(null, null);
        SmartDashboard.putNumber("DistanceToCenterHub", shooterSys.getPlanarDistanceToHubMeters());
        SmartDashboard.putString("Drive Mode", swerveSys.getCurrentMode().name());
        SmartDashboard.putBoolean("Is Aiming", aimToHubCmd.isScheduled());
        SmartDashboard.putBoolean("Vision Locked", LimelightHelpers.getTV(VisionConstants.LimelightName));
        SmartDashboard.putNumber("Pose Rotation", swerveSys.getPose().getRotation().getDegrees());
        SmartDashboard.putBoolean("Auto Aim Active", autoPointCmd.isScheduled());

        //System
        systemLayout.addNumber("Drive Voltage", () -> swerveSys.getAverageDriveVoltage());
        systemLayout.addNumber("Current Draw", () -> pdh.getTotalCurrent());
        systemLayout.addNumber("Battery Voltage", () -> RobotController.getBatteryVoltage());
        

        //DriveTrain
        drivetrainLayout.addNumber("FL Speed", () -> swerveSys.getModuleStates()[0].speedMetersPerSecond);
        drivetrainLayout.addNumber("FR Speed", () -> swerveSys.getModuleStates()[1].speedMetersPerSecond);
        drivetrainLayout.addNumber("BL Speed", () -> swerveSys.getModuleStates()[2].speedMetersPerSecond);
        drivetrainLayout.addNumber("BR Speed", () -> swerveSys.getModuleStates()[3].speedMetersPerSecond);
        drivetrainLayout.addNumber("Heading", () -> swerveSys.getHeading().getDegrees());
        drivetrainLayout.addNumber("Speed m/s", () -> swerveSys.getAverageDriveVelocityMetersPerSec());
        

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

        SmartDashboard.putNumber("Speed X", swerveSys.getFieldRelativeVelocity().getX());
        SmartDashboard.putNumber("Speed Y", swerveSys.getFieldRelativeVelocity().getY());
    }   
}
