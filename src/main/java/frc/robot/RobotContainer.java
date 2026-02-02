// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSimTest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //joystick and suppliers from joystick
    private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);

    private final Supplier<Double> stickFwdSupplier = () -> {return -joystick.getY()*MaxSpeed/2;};
    private final Supplier<Double> stickLeftSupplier = () -> {return -joystick.getX()*MaxSpeed/2;};

    //drivetrain and suppliers from drivetrain
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Supplier<Pose2d> poseSupplier = () -> {return drivetrain.getState().Pose;};
    private final Supplier<ChassisSpeeds> speedsSupplier = () -> {return drivetrain.getState().Speeds;};
    private final Supplier<Translation3d> targetSupplier = () -> {return drivetrain.our_hub;};

    //shooter and suppliers from shooter
    private final ShooterSimTest shooter = new ShooterSimTest(poseSupplier, speedsSupplier, targetSupplier);
    
    private final Supplier<Rotation2d> shotTargetYaw = () -> {return shooter.targetYaw();};



    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Configure the trigger bindings
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() 
    {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


        
        //joystick.button(8).onTrue(Commands.runOnce(() -> drivetrain.resetAllPoses(new Pose2d(0,0,new Rotation2d(0)))));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getY() * MaxSpeed/2) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getX() * MaxSpeed/2) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.button(11).whileTrue(drivetrain.autoAlign());

        joystick.button(2).whileTrue(
            drivetrain.aim(stickFwdSupplier, stickLeftSupplier, shotTargetYaw)
        );
        joystick.trigger().and(shooter.aimOk()).and(shooter.fireOk()).onTrue
        (
            shooter.fire()
        );
    
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.trigger().and(joystick.button(4)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.trigger().and(joystick.button(5)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.trigger().and(joystick.button(6)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.trigger().and(joystick.button(7)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        
        return autoChooser.getSelected();
    }
}
