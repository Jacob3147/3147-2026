package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.LocalizationConstants.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import static frc.robot.Constants.ShooterConstants.*;

/*
 * Simulating a rebuilt shooter with the goal of shoot on the fly capabilities 
 */
public class Shooter extends SubsystemBase 
{
    double shotVelocity;
    double azimuth;

    //shooter offset is from center of robot at floot
    Distance shooterHeight = Meters.of(0.37);
    Distance shooterOffsetX = Meters.of(-0.2);
    Distance shooterOffsetY = Meters.of(-0.1);
    Transform3d shooterOffset = new Transform3d(shooterOffsetX, shooterOffsetY,shooterHeight, new Rotation3d(SHOOTER_ROTATION));

    Time simTimestep = Milliseconds.of(20);

    //tried in vain to get an algebraic solution for shot speed and especially shot angle - turns out the answer is either empirical or use numerical methods instead of algebraic.
    //this lets you put as many ordered pairs as you want and then it will give a linear interpolation between them for any input
    InterpolatingDoubleTreeMap distance_to_speed = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap distance_to_azimuth = new InterpolatingDoubleTreeMap();
    
    boolean manualMode = false;
    public Trigger manualModeTrigger = new Trigger(() -> manualMode);
    boolean spinUp = false;

    Supplier<Pose2d> poseSupplier;
    Supplier<ChassisSpeeds> speedsSupplier;
    Supplier<Translation3d> targetSupplier;

    Pose2d robotPose;
    Pose3d shooterPose;

    Translation3d target;
    Translation3d robotToTarget;
    double targetDistance;

    LinearVelocity driveVelX;
    LinearVelocity driveVelY;

    DoublePublisher shotVelTargetPub = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/velocityTarget").publish();
    DoublePublisher shotAngleTargetPub = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/angleTarget").publish();
    DoublePublisher shotVelActPub = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/velocityActual").publish();
    DoublePublisher shotAngleActPub = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/angleActual").publish();
    DoublePublisher hubDistancePub = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/distanceToHub").publish();

    TalonFX shooter_1 = new TalonFX(40);
    TalonFX shooter_2 = new TalonFX(41);
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    Slot0Configs shooterSlot0 = shooterConfig.Slot0;
    CurrentLimitsConfigs shooterCurrentLimits = shooterConfig.CurrentLimits;
    MotionMagicConfigs shooterMotionConfigs = shooterConfig.MotionMagic;
    MotorOutputConfigs shooterOutputConfig = shooterConfig.MotorOutput;

    MotionMagicVelocityVoltage shooterRequest = new MotionMagicVelocityVoltage(0);


    TalonFXS hood = new TalonFXS(42);
    TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();
    Slot0Configs hoodSlot0 = hoodConfig.Slot0;
    CurrentLimitsConfigs hoodCurrentLimits = hoodConfig.CurrentLimits;
    MotionMagicConfigs hoodMotionConfigs = hoodConfig.MotionMagic;
    MotorOutputConfigs hoodOutputConfig = hoodConfig.MotorOutput;

    MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);


    TalonFXS indexer = new TalonFXS(30);
    TalonFXSConfiguration indexerConfig = new TalonFXSConfiguration();
    Slot0Configs indexerSlot0 = indexerConfig.Slot0;
    CurrentLimitsConfigs indexerCurrentLimits = indexerConfig.CurrentLimits;
    MotorOutputConfigs indexerOutputConfig = indexerConfig.MotorOutput;

    VoltageOut indexerRequest = new VoltageOut(0);

    
    //for bringup
    double shooter_pct = 0;
    double hood_pct = 0;
    double indexer_pct = 0;
    
    double shooter_kp = 0;
    double shooter_kd = 0;
    double shooter_kv = 0;//12 / Constants.KRAKEN_FREE_SPEED; //volt per rps

    VoltageOut shooterTestRequest = new VoltageOut(0);
    VoltageOut hoodTestRequest = new VoltageOut(0);


    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier, Supplier<Translation3d> targetSupplier) 
    {
        shooterSlot0.withKS(0)
                    .withKV(shooter_kv)
                    .withKA(0)
                    .withKP(0)
                    .withKI(0)
                    .withKD(0);
        shooterCurrentLimits.StatorCurrentLimit = 60;
        shooterMotionConfigs.MotionMagicAcceleration = 100;
        shooterOutputConfig.NeutralMode = NeutralModeValue.Coast;
        shooterOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        shooter_1.getConfigurator().apply(shooterConfig);
        shooter_2.getConfigurator().apply(shooterConfig);
        shooter_1.setControl(shooterRequest);
        shooter_2.setControl(new StrictFollower(shooter_1.getDeviceID()));


        hoodSlot0.withKS(0)
                 .withKV(0)
                 .withKA(0)
                 .withKP(0)
                 .withKI(0)
                 .withKD(0);
        hoodCurrentLimits.StatorCurrentLimit = 20;
        hoodMotionConfigs.MotionMagicAcceleration = 100;
        hoodOutputConfig.NeutralMode = NeutralModeValue.Coast;
        hoodOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        hood.getConfigurator().apply(hoodConfig);
        hood.setControl(hoodRequest);


        indexerSlot0.withKS(0)
                 .withKV(0)
                 .withKA(0)
                 .withKP(0)
                 .withKI(0)
                 .withKD(0);
        indexerCurrentLimits.StatorCurrentLimit = 20;
        indexerOutputConfig.NeutralMode = NeutralModeValue.Coast;
        indexerOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        indexer.getConfigurator().apply(indexerConfig);
        indexer.setControl(indexerRequest);
        

        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.targetSupplier = targetSupplier;

        //sending data to network tables to visualize in advantagescope


       
        //distances to note:
        //1.375 seems to be closest possible shot
        //5.5 is about the longest shot I could see

        //map distance from goal to shot speed
        /*distance_to_speed.put(0.0,6.0);
        distance_to_speed.put(1.375,6.0);
        distance_to_speed.put(1.5,6.01);
        distance_to_speed.put(2.0,6.38);
        distance_to_speed.put(2.5,6.75);
        distance_to_speed.put(3.0,7.12);
        distance_to_speed.put(3.5,7.3);
        distance_to_speed.put(4.0,7.44);
        distance_to_speed.put(4.5,7.78);
        distance_to_speed.put(5.0,8.13);
        distance_to_speed.put(5.5,8.47);
        distance_to_speed.put(8.0,8.47);

        

        //map distance from goal to shot angle (angles are measured above the horizontal)
        distance_to_azimuth.put(0.0,75.0);
        distance_to_azimuth.put(1.375,75.0);
        distance_to_azimuth.put(1.5,74.38); 
        distance_to_azimuth.put(2.0,71.50);
        distance_to_azimuth.put(2.5, 68.17);
        distance_to_azimuth.put(3.0,65.90);
        distance_to_azimuth.put(3.5,62.76);
        distance_to_azimuth.put(4.0,61.17);
        distance_to_azimuth.put(4.6,60.0);
        distance_to_azimuth.put(8.0,60.0);*/
        
        SmartDashboard.putNumber("bringup/shooter pct",0);
        SmartDashboard.putNumber("bringup/hood pct",0);
        SmartDashboard.putNumber("bringup/indexer pct",0);
        SmartDashboard.putBoolean("bringup/zero hood", false);
        SmartDashboard.putNumber("bringup/shooter kp", 0);
        SmartDashboard.putNumber("bringup/shooter kd", 0);
    }

    @Override
    public void periodic() 
    {
        //bringup
        if(SmartDashboard.getBoolean("bringup/zero hood", false)) hood.setPosition(0);
        shooter_pct = SmartDashboard.getNumber("bringup/shooter pct",0);
        hood_pct = SmartDashboard.getNumber("bringup/hood pct",0);
        indexer_pct = SmartDashboard.getNumber("bringup/indexer pct",0);
        shooter_kp = SmartDashboard.getNumber("bringup/shooter kp", 0);
        shooter_kd = SmartDashboard.getNumber("bringup/shooter kd", 0);
        shooter_1.setControl(shooterTestRequest);
        shooterTestRequest.withOutput(shooter_pct * 12);

        hood.setControl(hoodTestRequest);
        hoodTestRequest.withOutput(hood_pct * 12);

        indexer.setControl(indexerRequest);
        indexerRequest.withOutput(indexer_pct * 12);

        /*
        shooter_1.setControl(shooterRequest);
        shooterSlot0.withKP(shooter_kp).withKD(shooter_kd);
        shooter_1.getConfigurator().apply(shooterConfig);
        shooterRequest.withVelocity(shooter_pct);*/


        //calculate this continuously so we can tell the drivetrain what yaw to target

        //positions of robot and hub
        robotPose = poseSupplier.get();
        target = targetSupplier.get();

        //3d position of shooter
        shooterPose = new Pose3d(robotPose).plus(shooterOffset);

        //points from shooter to target
        robotToTarget = shooterPose.getTranslation().minus(target);

        //distance from shooter to target
        targetDistance = Math.sqrt(Math.pow(robotToTarget.getX(),2) + Math.pow(robotToTarget.getY(),2));
        
        //intended velocity and azimuth (pitch) from lookup table based on distance
        if(!manualMode)
        {
            //shotVelocity = distance_to_speed.get(targetDistance);
            //azimuth = distance_to_azimuth.get(targetDistance);
            azimuth = MANUAL_AZIMUTH;
            shotVelocity = MANUAL_VELOCITY;
        }
        else
        {
            azimuth = MANUAL_AZIMUTH;
            shotVelocity = MANUAL_VELOCITY;
        }

        shotVelTargetPub.set(shotVelocity);
        shotVelActPub.set(shooter_1.getVelocity(true).getValueAsDouble());
        shotAngleTargetPub.set(azimuth);
        shotAngleActPub.set(hood.getPosition(true).getValueAsDouble());
    }

    public void shooterControl(double rpm_target)
    {
        shooterRequest.withVelocity(rpm_target/60);
    }

    public void hoodControl(double rotations_target)
    {
        if(rotations_target > HOOD_MAX_ROTATIONS) rotations_target = HOOD_MAX_ROTATIONS;
        hoodRequest.withPosition(rotations_target);
    }

    public void runIndexer()
    {
        indexerRequest.withOutput(3);
    }
    public void stopIndexer()
    {
        indexerRequest.withOutput(0);
    }

    public Rotation2d targetYaw()
    {
        return new Rotation2d(robotToTarget.getX(), robotToTarget.getY()).plus(Rotation2d.k180deg).minus(SHOOTER_ROTATION);
    } 

    public Trigger aimOk()
    {
        return new Trigger(() -> 
        (
            poseSupplier.get().getRotation().minus(targetYaw()).getDegrees() < 2
            || manualMode
        ));
    }

    
    public void manualOn()
    {
        manualMode = true;
    }
    public void manualOff()
    {
        manualMode = false;
    }

    public void spinUp()
    {
        spinUp = true;
    }
    public void spinDown()
    {
        spinUp = false;
    }


    public Command fire()
    {
        return Commands.startEnd
        (
            () -> {runIndexer();},
            () -> {stopIndexer();}
        );
    }
}