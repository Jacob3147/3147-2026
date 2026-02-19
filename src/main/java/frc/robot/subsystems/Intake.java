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

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{   
    double tilt_ratio = 20;


    TalonFX tilt = new TalonFX(20);
    TalonFXConfiguration tiltConfig = new TalonFXConfiguration();
    Slot0Configs tiltSlot0 = tiltConfig.Slot0;
    CurrentLimitsConfigs tiltCurrentLimits = tiltConfig.CurrentLimits;
    MotionMagicConfigs tiltMotionConfigs = tiltConfig.MotionMagic;
    MotorOutputConfigs tiltOutputConfig = tiltConfig.MotorOutput;
    FeedbackConfigs tiltFeedbackConfigs = tiltConfig.Feedback;

    MotionMagicExpoVoltage tiltRequest = new MotionMagicExpoVoltage(0);

    TalonFX wheels = new TalonFX(21);
    TalonFXConfiguration wheelsConfig = new TalonFXConfiguration();
    Slot0Configs wheelsSlot0 = wheelsConfig.Slot0;
    CurrentLimitsConfigs wheelsCurrentLimits = wheelsConfig.CurrentLimits;
    MotorOutputConfigs wheelsOutputConfig = wheelsConfig.MotorOutput;

    VoltageOut wheelsRequest = new VoltageOut(0);

    DoublePublisher tiltAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Tilt angle").publish();
    //bringup
    double wheels_pct = 0;
    double tilt_pct = 0;

    double tilt_sp = 0;
    double tilt_kp = 0;
    double tilt_kd = 0;
    double tilt_kv = 0;//12 / (Constants.KRAKEN_FREE_SPEED / tilt_ratio); //volt per (rps of end effector)

    VoltageOut tiltRequestTest = new VoltageOut(0);

    
    public Intake()
    {
         tiltSlot0.withKS(0)
                 .withKV(0)
                 .withKA(0)
                 .withKP(0)
                 .withKI(0)
                 .withKD(0);
        
        tiltCurrentLimits.StatorCurrentLimit = 20;
        tiltMotionConfigs.MotionMagicAcceleration = 100;
        tiltOutputConfig.NeutralMode = NeutralModeValue.Coast;
        tiltFeedbackConfigs.SensorToMechanismRatio = tilt_ratio;
        tiltOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        tilt.getConfigurator().apply(tiltConfig);
        tilt.setControl(tiltRequest);

        wheelsSlot0.withKS(0)
                 .withKV(0)
                 .withKA(0)
                 .withKP(0)
                 .withKI(0)
                 .withKD(0);
        wheelsCurrentLimits.StatorCurrentLimit = 20;
        wheelsOutputConfig.NeutralMode = NeutralModeValue.Coast;
        wheelsOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        wheels.getConfigurator().apply(wheelsConfig);
        wheels.setControl(wheelsRequest);

        SmartDashboard.putNumber("bringup/tilt pct", 0);
        SmartDashboard.putNumber("bringup/wheels pct", 0);
        SmartDashboard.putBoolean("bringup/zero intake", false);
        SmartDashboard.putNumber("bringup/tilt kp", 0);
        SmartDashboard.putNumber("bringup/tilt kd", 0);
    }    

    @Override
    public void periodic() 
    {   
        //bringup
        if(SmartDashboard.getBoolean("bringup/zero intake", false)) tilt.setPosition(0);
        tilt_pct = SmartDashboard.getNumber("bringup/tilt pct", 0);
        wheels_pct = SmartDashboard.getNumber("bringup/wheels pct", 0);
        tilt_kp = SmartDashboard.getNumber("bringup/tilt kp", 0);
        tilt_kd = SmartDashboard.getNumber("bringup/tilt kd", 0);
        tilt.setControl(tiltRequestTest);
        tiltRequestTest.withOutput(tilt_pct * 12);

        wheels.setControl(wheelsRequest);
        wheelsRequest.withOutput(wheels_pct * 12);
        
        /*
        tilt.setControl(tiltRequest);
        tiltSlot0.withKP(tilt_kp).withKD(tilt_kd);
        tilt.getConfigurator().apply(tiltConfig);
        tiltRequest.withPosition(tilt_pct);*/




        tiltAnglePublisher.set(tilt.getPosition(true).getValueAsDouble());
    }
    public Command deploy()
    {
        return Commands.startEnd(
        () -> {

        },
        () -> {
            
        },
        this
        );
    }
}
