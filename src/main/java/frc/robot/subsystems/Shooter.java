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
import com.ctre.phoenix6.signals.MotorArrangementValue;
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
    double rps_target;
    double hood_target;

    //shooter offset is from center of robot at floot
    Distance shooterHeight = Meters.of(0.37);
    Distance shooterOffsetX = Meters.of(-0.2);
    Distance shooterOffsetY = Meters.of(-0.1);
    Transform3d shooterOffset = new Transform3d(shooterOffsetX, shooterOffsetY,shooterHeight, new Rotation3d(SHOOTER_ROTATION));

    Time simTimestep = Milliseconds.of(20);

    //tried in vain to get an algebraic solution for shot speed and especially shot angle - turns out the answer is either empirical or use numerical methods instead of algebraic.
    //this lets you put as many ordered pairs as you want and then it will give a linear interpolation between them for any input
    InterpolatingDoubleTreeMap distance_to_rps = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap distance_to_hood = new InterpolatingDoubleTreeMap();
    
    boolean manualMode = true;
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

    TalonFX shooter_1 = new TalonFX(20);
    TalonFX shooter_2 = new TalonFX(21);
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    Slot0Configs shooterSlot0 = shooterConfig.Slot0;
    CurrentLimitsConfigs shooterCurrentLimits = shooterConfig.CurrentLimits;
    MotionMagicConfigs shooterMotionConfigs = shooterConfig.MotionMagic;
    MotorOutputConfigs shooterOutputConfig = shooterConfig.MotorOutput;


    MotionMagicVelocityVoltage shooterRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    


    TalonFXS hood = new TalonFXS(24);
    TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();
    Slot0Configs hoodSlot0 = hoodConfig.Slot0;
    CurrentLimitsConfigs hoodCurrentLimits = hoodConfig.CurrentLimits;
    MotionMagicConfigs hoodMotionConfigs = hoodConfig.MotionMagic;
    MotorOutputConfigs hoodOutputConfig = hoodConfig.MotorOutput;

    MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);

    double hood_kv = 12.0 / (Constants.MINION_FREE_SPEED);
    double hood_ks = 0.18;
    double hood_kp = 0.4
    ;
    double hood_kd = 0;


    TalonFXS indexer = new TalonFXS(22);
    TalonFXSConfiguration indexerConfig = new TalonFXSConfiguration();
    Slot0Configs indexerSlot0 = indexerConfig.Slot0;
    CurrentLimitsConfigs indexerCurrentLimits = indexerConfig.CurrentLimits;
    MotorOutputConfigs indexerOutputConfig = indexerConfig.MotorOutput;

    VoltageOut indexerRequest = new VoltageOut(0);

    
    double shooter_kp = 0.75;
    double shooter_kv = 11.75 / Constants.KRAKEN_FREE_SPEED;

    //VoltageOut shooterRequest2 = new VoltageOut(0);

    double shooter_rps_target = 53;


    double manualVoltage = 0;
    double shooterVoltage = 0;

    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier, Supplier<Translation3d> targetSupplier) 
    {
        shooterSlot0.withKS(0)
                    .withKV(shooter_kv)
                    .withKA(0)
                    .withKP(shooter_kp)
                    .withKI(0)
                    .withKD(0);
        shooterCurrentLimits.StatorCurrentLimit = 60;
        shooterMotionConfigs.MotionMagicAcceleration = 100;
        shooterOutputConfig.NeutralMode = NeutralModeValue.Coast;
        shooterOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
        shooter_1.getConfigurator().apply(shooterConfig);
        shooter_2.getConfigurator().apply(shooterConfig);
        shooterRequest.withAcceleration(500);
        shooter_1.setControl(shooterRequest);
        shooter_2.setControl(new StrictFollower(shooter_1.getDeviceID()));


        hoodSlot0.withKS(hood_ks)
                 .withKV(hood_kv)
                 .withKA(0)
                 .withKP(hood_kp)
                 .withKI(0)
                 .withKD(hood_kd);
        hoodCurrentLimits.StatorCurrentLimit = 60;
        hoodMotionConfigs.MotionMagicAcceleration = 40;
        hoodMotionConfigs.MotionMagicCruiseVelocity = 40;
        hoodOutputConfig.NeutralMode = NeutralModeValue.Coast;
        hoodOutputConfig.PeakForwardDutyCycle = 0.7;
        hoodOutputConfig.PeakReverseDutyCycle = -0.7;
        hoodOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        hood.getConfigurator().apply(hoodConfig);
        hood.setControl(hoodRequest);


        indexerSlot0.withKS(0)
                 .withKV(0)
                 .withKA(0)
                 .withKP(0)
                 .withKI(0)
                 .withKD(0);
        indexerCurrentLimits.StatorCurrentLimit = 60;
        indexerOutputConfig.NeutralMode = NeutralModeValue.Coast;
        indexerOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        indexerConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        indexer.getConfigurator().apply(indexerConfig);
        indexer.setControl(indexerRequest);
        

        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.targetSupplier = targetSupplier;

        //sending data to network tables to visualize in advantagescope
        

        //map distance from goal to shot speed
        distance_to_rps.put(0.0,58.0);
        distance_to_rps.put(1.5,58.0);
        distance_to_rps.put(2.0,58.0);
        distance_to_rps.put(2.4,58.0);
        distance_to_rps.put(2.81,60.0);
        distance_to_rps.put(3.24,63.0);
        distance_to_rps.put(3.77,65.0);
        distance_to_rps.put(4.23,67.0);
        distance_to_rps.put(5.16,74.0);
        distance_to_rps.put(7.0,74.0);

        

        //map distance from goal to hood rotations
        distance_to_hood.put(0.0,0.0);
        distance_to_hood.put(1.5,0.0);
        distance_to_hood.put(2.0,0.0);
        distance_to_hood.put(2.4,3.0);
        distance_to_hood.put(2.81,4.5);
        distance_to_hood.put(3.24,6.0);
        distance_to_hood.put(3.77,7.88);
        distance_to_hood.put(4.23,8.8);
        distance_to_hood.put(5.16,10.88);
        distance_to_hood.put(7.0,10.88);
        
        SmartDashboard.putNumber("bringup/shooter pct",58);
        SmartDashboard.putBoolean("bringup/override shooter", false);
        SmartDashboard.putBoolean("bringup/zero hood", false);
        

        
    }

    @Override
    public void periodic() 
    {
        manualVoltage = SmartDashboard.getNumber("bringup/shooter pct", 0);
        SmartDashboard.putNumber("bringup/hood angle", hood.getPosition(true).getValueAsDouble());
        //bringup
        if(SmartDashboard.getBoolean("bringup/zero hood", false)) hood.setPosition(0);

        shooter_1.setControl(shooterRequest);
        hood.setControl(hoodRequest);
        indexer.setControl(indexerRequest);


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
        SmartDashboard.putNumber("target distance", targetDistance);
       

        //intended velocity and hood_target (pitch) from lookup table based on distance
        if(!manualMode)
        {
            hood_target = distance_to_hood.get(targetDistance);
            rps_target = distance_to_rps.get(targetDistance);
            
        }
        else
        {
            hood_target = 0;
            rps_target = manualVoltage;
        }
        SmartDashboard.putBoolean("auto shot", !manualMode);
        SmartDashboard.putNumber("hood target", hood_target);
        SmartDashboard.putNumber("rps target", rps_target);
        shotVelTargetPub.set(rps_target);
        shotVelActPub.set(shooter_1.getVelocity(true).getValueAsDouble());
        shotAngleTargetPub.set(hood_target);
        shotAngleActPub.set(hood.getPosition(true).getValueAsDouble());

        
        if(spinUp)
        {
            shooterRequest.withVelocity(/*rps_target*/0);
            hoodRequest.withPosition(hood_target);
            //shooterRequest2.withOutput(rps_target);
        }
        else
        {
            shooterRequest.withVelocity(0);
            hoodRequest.withPosition(0);
            //shooterRequest2.withOutput(0);
        }
    }

    public Command indexer()
    {
        return Commands.startEnd(
        () -> {
            indexerRequest.withOutput(4.5);
        },
        () -> {
            indexerRequest.withOutput(0);
        },
        this
        );
    }

    public void spinUp() { spinUp = true;}
    public void spinDown() { spinUp = false; }

    public void manualShooter() { manualMode = true;}
    public void autoShooter() { manualMode = false; }
    

    public Trigger safeTofire = new Trigger(() -> shooter_1.getVelocity(true).getValueAsDouble() > 40);



    public Rotation2d targetYaw()
    {
        return new Rotation2d(robotToTarget.getX(), robotToTarget.getY()).plus(SHOOTER_ROTATION);
    } 

    public Trigger aimOk()
    {
        return new Trigger(() -> 
        (
            poseSupplier.get().getRotation().minus(targetYaw()).getDegrees() < 2
            || manualMode
        ));
    }

    


    



}