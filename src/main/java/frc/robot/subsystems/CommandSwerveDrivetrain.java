package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.LocalizationConstants.*;
import static frc.robot.Constants.AutopilotConstants.*;

import frc.robot.utility.Autopilot.*;
import frc.robot.utility.Autopilot.Autopilot.APResult;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightSettings.LEDMode;
import static edu.wpi.first.units.Unit.*;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    

    QuestNav questNav;
    Limelight limelight;

    Pose3d quest_camera_pose;
    double quest_timestamp;
    Pose3d quest_robot_pose;

    Pose3d limelight_camera_pose;
    double limelight_timestamp;
    Pose3d limelight_robot_pose;
    LimelightPoseEstimator limelightPoseEstimator;
    
    APTarget autopilotTarget = new APTarget(
                                    new Pose2d(
                                        Meters.of(5.28), 
                                        Meters.of(2.84), 
                                        new Rotation2d(Degrees.of(120))
                                    )
                                ).withEntryAngle(new Rotation2d(Degrees.of(120)));

    private final SwerveRequest.FieldCentricFacingAngle m_autopilotRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(2,0,0)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance) //use field perspective rather than driver station perspective
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.Idle stopRequest = new SwerveRequest.Idle();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) 
    {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }
        
        questNav = new QuestNav();
        limelight = new Limelight("limelight");
        limelight.getSettings()
                 .withLimelightLEDMode(LEDMode.ForceOff)
                 .withCameraOffset(ROBOT_TO_LIMELIGHT)
                 .save();
        limelightPoseEstimator = limelight.getPoseEstimator(true);

        configureAutoBuilder();

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) 
    {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() 
    {
        try {
            //var config = RobotConfig.fromGUISettings();
            RobotConfig config = new RobotConfig
                                (
                                    Pounds.of(150), //find exact later - this is approx max weight plus bumper and battery
                                    MomentOfInertia.ofBaseUnits(6, KilogramSquareMeters),   //find exact later
                                    new ModuleConfig
                                    (
                                        TunerConstants.kWheelRadius,
                                        TunerConstants.kSpeedAt12Volts,
                                        1.2,
                                        DCMotor.getKrakenX60(1),
                                        6.75,
                                        TunerConstants.kSlipCurrent,
                                        1
                                    ),
                                    new Translation2d[] 
                                    {
                                        TunerConstants.kFrontLeftPos,
                                        TunerConstants.kFrontRightPos,
                                        TunerConstants.kBackLeftPos,
                                        TunerConstants.kBackRightPos
                                    }
                                 );
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetAllPoses,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

   

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        periodicVisionTasks();
        
    }

    public Command autoAlign(/*APTarget target*/) {
        return this.run(() -> {
            ChassisSpeeds robotRelativeSpeeds = this.getState().Speeds;
            Pose2d pose = this.getState().Pose;
            

            APResult output = kAutopilot.calculate(pose, robotRelativeSpeeds, autopilotTarget);
        
            /* these speeds are field relative */
            LinearVelocity veloX = output.getX();
            LinearVelocity veloY = output.getY();
            Rotation2d headingReference = output.getHeading();
        
            this.setControl(m_autopilotRequest
                .withVelocityX(veloX)
                .withVelocityY(veloY)
                .withTargetDirection(headingReference));
            })
                .until(() -> kAutopilot.atTarget(this.getState().Pose, autopilotTarget));
      }


    /***********************************
     * Localization code
     ***********************************

    * FRC uses the following coordinate system:
    * 
    * On-Robot:
    * Origin (0,0,0) is the center of the robot, on the floor
    * 
    * On-Field:
    * Origin (0,0,0) is the blue driver's station, on the right side of the field from the driver's perspective, on the floor * 
    * 
    * Relative directions are the same for both:
    * +X = forward 
    * +Y = left
    * +Z = up
    * +yaw = CCW rotation about Z axis (turning left)
    * +pitch = CCW rotation about Y axis (back bumper raising off floor, or nose down)
    * +roll = CCW rotation about X axis (left bumper raising off floor) 
    * 
    * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    */

    private void periodicVisionTasks()
    {
        //required to be called constantly for QuestNav
        questNav.commandPeriodic(); 
        
        //required to be called constantly for LimeLight
        limelight.getSettings()
                 .withRobotOrientation(
                    new Orientation3d(
                        getPigeon2().getRotation3d(),
                        new AngularVelocity3d(
                            getPigeon2().getAngularVelocityXWorld(true).getValue(),
                            getPigeon2().getAngularVelocityYWorld(true).getValue(),
                            getPigeon2().getAngularVelocityZWorld(true).getValue()
                        )
                    )
                )
                 .save();


        getPoseFromQuestNav();
        getPoseFromLimelight();
    }

    
    private void getPoseFromQuestNav()
    {
        if(questNav.isTracking())
        {
            // Get latest data from Quest
            PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

            // Loop over pose data frames
            for(PoseFrame questFrame : questFrames)
            {
                quest_camera_pose = questFrame.questPose3d();
                quest_timestamp = questFrame.dataTimestamp();

                quest_robot_pose = quest_camera_pose.transformBy(ROBOT_TO_QUEST.inverse());

                //any filtering goes here

                //add to pose estimator
                addVisionMeasurement(quest_robot_pose.toPose2d(), quest_timestamp, QUESTNAV_STD_DEVS);
            }
        }

    }

    private void getPoseFromLimelight()
    {
        Optional<PoseEstimate> visionEstimate = limelightPoseEstimator.getPoseEstimate();
        // If the pose is present
        visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
            // If the average tag distance is less than 4 meters,
            // there are more than 0 tags in view,
            // and the average ambiguity between tags is less than 30% then we update the pose estimation.
            if (poseEstimate.avgTagDist < 4 && poseEstimate.tagCount > 0 && poseEstimate.getMinTagAmbiguity() < 0.3)
            {
                addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds, LIMELIGHT_STD_DEVS);
            }
          });
    }

    
    public void resetAllPoses(Pose2d pose)
    {
        super.resetPose(pose);  //drivetrain
        resetQuestPose(pose);   //quest
        resetLimelightPose(pose);   //LL is absolute only so this should be irrelevant
    }


    private void resetQuestPose(Pose2d pose)
    {
        questNav.setPose(new Pose3d(pose).transformBy(ROBOT_TO_QUEST));
    }

    private void resetLimelightPose(Pose2d pose)
    {

    }


    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }







    /*
     * premade sysid routines for characterization
     * sysid consists of 4 tests - dynamic and quasistatic, forwards and backwards. this will give you PID+FF gains
     * this has 3 different routines which each implement the 4 tests. they are for drive motors, steering motors, and heading
     * change m_sysIdRoutineToApply to determine which routine is used. the 4 tests are called as commands in robotcontainer.
     */

     

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

     /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;



    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
}


}