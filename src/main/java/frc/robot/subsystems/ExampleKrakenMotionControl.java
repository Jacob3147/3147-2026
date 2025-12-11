package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleKrakenMotionControl extends SubsystemBase
{
    TalonFX kraken;
    TalonFXConfiguration krakenConfig;
    MotionMagicExpoVoltage motionRequest;


    public ExampleKrakenMotionControl()
    {
        // Define motor at CAN ID
        kraken = new TalonFX(0);

        // Define a configuration variable, which has many different components that we break out into their own variables
        krakenConfig = new TalonFXConfiguration();
        Slot0Configs krakenSlot0 = krakenConfig.Slot0;
        CurrentLimitsConfigs krakenCurrentLimits = krakenConfig.CurrentLimits;
        MotionMagicConfigs krakenMotionConfigs = krakenConfig.MotionMagic;
        FeedbackConfigs krakenFeedback = krakenConfig.Feedback;
        MotorOutputConfigs krakenOutputConfig = krakenConfig.MotorOutput;

        /* 
        * Gravity type is one of:
        * None (like a drive wheel)
        * Elevator static (constantly fighting gravity, but always the same)
        * Arm cosine (fighting gravity but changes based on the angle)

        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/index.html
        */
        krakenSlot0.withGravityType(GravityTypeValue.Elevator_Static)
                    // Feed forward values - predictive control
                   .withKA(0)   // voltage to achieve a certain acceleration - typically will be left at 0
                   .withKV(0)   // voltage to achieve a certain velocity (12 / max speed)
                   .withKS(0)   // the lowest voltage that overcomes mechanism friction
                   .withKG(0)   // the voltage that exactly overcomes gravity
                   // Feedback values - reactive control
                   .withKP(0)   // an error of 1 rotation results in this many volts of output
                   .withKI(0)   // typically left at 0 - an error of 1 rotation for 1 second results in this much added to output
                   .withKD(0);  // a velocity error of 1 rotation per second results in this many volts of output
        krakenCurrentLimits.withStatorCurrentLimit(60);
        krakenMotionConfigs.withMotionMagicAcceleration(1)
                           .withMotionMagicCruiseVelocity(1);
        krakenFeedback.withSensorToMechanismRatio(50);
        krakenOutputConfig.NeutralMode = NeutralModeValue.Brake;
        krakenOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

        kraken.getConfigurator().apply(krakenConfig);
        
        kraken.setPosition(0);
    }

    @Override
    public void periodic() 
    {
        //run to position
        motionRequest.withPosition(0);
    }
}
