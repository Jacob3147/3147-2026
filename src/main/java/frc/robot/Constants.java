// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.math.util.Units.*;
import frc.robot.utility.Autopilot.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static class OperatorConstants 
    {
        public static final int kDriverControllerPort = 0;
    }

    public static class LocalizationConstants
    {
        //quest offset from center of robot
        public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
            new Translation3d(
                0,
                0,
                0
            ),
            new Rotation3d(
                0,
                0,
                0
            )
        );

        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.02,   //trust pose to 2 cm in X and Y
            0.02,
            degreesToRadians(2)   //trust pose to 2 degrees
        );

        public static final Pose3d ROBOT_TO_LIMELIGHT = new Pose3d(
            new Translation3d(
                0,
                0,
                0
            ),
            new Rotation3d(
                0,
                0,
                0
            )
        );

        public static final Matrix<N3, N1> LIMELIGHT_STD_DEVS = VecBuilder.fill(
            0.2,   //trust pose to 20cm in X and Y (subject to change or may be dynamic)
            0.2,
            degreesToRadians(999)   //ignore rotation
        );
    }

    public static class AutopilotConstants
    {
        private static final APConstraints kConstraints = new APConstraints(7.0, 3.0);

        private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Inches.of(1))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Inches.of(4));

        public static final Autopilot kAutopilot = new Autopilot(kProfile);
    }
}
