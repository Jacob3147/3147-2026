package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.StrictFollower;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase
{   
    double tilt_ratio = 9;


    TalonFX tilt_left = new TalonFX(52);
    TalonFXConfiguration tilt_leftConfig = new TalonFXConfiguration();
    Slot0Configs tilt_leftSlot0 = tilt_leftConfig.Slot0;
    CurrentLimitsConfigs tilt_leftCurrentLimits = tilt_leftConfig.CurrentLimits;
    MotionMagicConfigs tilt_leftMotionConfigs = tilt_leftConfig.MotionMagic;
    MotorOutputConfigs tilt_leftOutputConfig = tilt_leftConfig.MotorOutput;
    FeedbackConfigs tilt_leftFeedbackConfigs = tilt_leftConfig.Feedback;

    TalonFX tilt_right = new TalonFX(51);
    TalonFXConfiguration tilt_rightConfig = new TalonFXConfiguration();
    Slot0Configs tilt_rightSlot0 = tilt_rightConfig.Slot0;
    CurrentLimitsConfigs tilt_rightCurrentLimits = tilt_rightConfig.CurrentLimits;
    MotionMagicConfigs tilt_rightMotionConfigs = tilt_rightConfig.MotionMagic;
    MotorOutputConfigs tilt_rightOutputConfig = tilt_rightConfig.MotorOutput;
    FeedbackConfigs tilt_rightFeedbackConfigs = tilt_rightConfig.Feedback;

    

    MotionMagicExpoVoltage tiltRequest = new MotionMagicExpoVoltage(0);
    VoltageOut tiltJogRequest = new VoltageOut(0.5);

    TalonFX wheels = new TalonFX(50);
    TalonFXConfiguration wheelsConfig = new TalonFXConfiguration();
    Slot0Configs wheelsSlot0 = wheelsConfig.Slot0;
    CurrentLimitsConfigs wheelsCurrentLimits = wheelsConfig.CurrentLimits;
    MotorOutputConfigs wheelsOutputConfig = wheelsConfig.MotorOutput;

    VoltageOut wheelsRequest = new VoltageOut(0);

    DoublePublisher tiltAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Tilt angle").publish();


    double intake_deployed_pos = 3;


    double tilt_kp = 0;
    double tilt_kd = 0;
    double tilt_kv = 12 / (Constants.KRAKEN_FREE_SPEED / tilt_ratio); //volt per (rps of end effector)

    boolean deploy = false;
    
    public Intake()
    {
         tilt_leftSlot0.withKS(0)
                 .withKV(tilt_kv)
                 .withKA(0)
                 .withKP(0)
                 .withKI(0)
                 .withKD(0);
        
        tilt_leftCurrentLimits.StatorCurrentLimit = 60;
        tilt_leftMotionConfigs.MotionMagicAcceleration = 50;
        tilt_leftOutputConfig.NeutralMode = NeutralModeValue.Coast;
        tilt_leftFeedbackConfigs.SensorToMechanismRatio = tilt_ratio;
        tilt_leftOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        tilt_left.getConfigurator().apply(tilt_leftConfig);
        tilt_left.setControl(tiltRequest);

        tilt_rightSlot0.withKS(0)
                 .withKV(tilt_kv)
                 .withKA(0)
                 .withKP(0)
                 .withKI(0)
                 .withKD(0);
        
        tilt_rightCurrentLimits.StatorCurrentLimit = 60;
        tilt_rightMotionConfigs.MotionMagicAcceleration = 50;
        tilt_rightOutputConfig.NeutralMode = NeutralModeValue.Coast;
        tilt_rightFeedbackConfigs.SensorToMechanismRatio = tilt_ratio;
        tilt_rightOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
        tilt_right.getConfigurator().apply(tilt_rightConfig);

        wheelsSlot0.withKS(0)
                 .withKV(0)
                 .withKA(0)
                 .withKP(0)
                 .withKI(0)
                 .withKD(0);
        wheelsCurrentLimits.StatorCurrentLimit = 80;
        wheelsOutputConfig.NeutralMode = NeutralModeValue.Coast;
        wheelsOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        wheels.getConfigurator().apply(wheelsConfig);
        wheels.setControl(wheelsRequest);

        SmartDashboard.putBoolean("bringup/zero intake", false);
    }    

    @Override
    public void periodic() 
    {   
        if(SmartDashboard.getBoolean("bringup/zero intake", false))
        {
            tilt_left.setPosition(0);
            tilt_right.setPosition(0);
        }
        tilt_left.setControl(tiltRequest);
        tilt_right.setControl(tiltRequest);

        wheels.setControl(wheelsRequest);
        
        if(deploy)
        {
            tiltRequest.withPosition(intake_deployed_pos);
        }
        else
        {
            tiltRequest.withPosition(0);
        }

        tiltAnglePublisher.set(tilt_left.getPosition(true).getValueAsDouble());
    }


    public Command deploy()
    {
        return Commands.runOnce(() -> {deploy = true;});
    }

    public Command retract()
    {
        return Commands.runOnce(() -> {deploy = false;});
    }


    boolean deployPrev;
    public Command spin()
    {
        return Commands.startEnd(
        () -> {
            deployPrev = deploy;
            deploy = true;
            wheelsRequest.withOutput(5);
        },
        () -> {
            deploy = deployPrev;
            wheelsRequest.withOutput(0);
        },
        this
        );
    }


    public Command pulse()
    {
        
        return Commands.repeatingSequence
        (
            
            Commands.runOnce(() -> 
            {
                deploy = false;
            }),
            Commands.waitSeconds(1),
            Commands.runOnce(() ->
            {
                deploy = true;
            })
        ).andThen
        (
            Commands.runOnce(() -> {deploy = deployPrev;})
        );
    }




    public Command reverse()
    {
        return Commands.startEnd(
        () -> {
            wheelsRequest.withOutput(-3);
        },
        () -> {
            wheelsRequest.withOutput(0);
        },
        this
        );
    }

    public Command turbo()
    {
        return Commands.startEnd(
        () -> {
            wheelsRequest.withOutput(12);
        },
        () -> {
            wheelsRequest.withOutput(0);
        },
        this
        );
    }

}
