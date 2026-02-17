package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{   
    TalonFX tilt = new TalonFX(20);
    TalonFXConfiguration tiltConfig;
    Slot0Configs tiltSlot0 = tiltConfig.Slot0;
    CurrentLimitsConfigs tiltCurrentLimits = tiltConfig.CurrentLimits;
    MotionMagicConfigs tiltMotionConfigs = tiltConfig.MotionMagic;
    MotorOutputConfigs tiltOutputConfig = tiltConfig.MotorOutput;

    TalonFX wheels = new TalonFX(21);
    TalonFXConfiguration wheelsConfig;
    Slot0Configs wheelsSlot0 = wheelsConfig.Slot0;
    CurrentLimitsConfigs wheelsCurrentLimits = wheelsConfig.CurrentLimits;
    MotorOutputConfigs wheelsOutputConfig = wheelsConfig.MotorOutput;


    public Intake()
    {

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
