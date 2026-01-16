package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.LocalizationConstants.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/*
 * Simulating a rebuilt shooter with the goal of shoot on the fly capabilities 
 */
public class ShooterSimTest extends SubsystemBase 
{
    LinearVelocity shotVelocity;
    Angle azimuth ;
    LinearAcceleration gravity = MetersPerSecondPerSecond.of(-9.8);

    //shooter offset is from center of robot at floot
    Distance shooterHeight = Meters.of(0.5);
    Distance shooterOffsetX = Meters.of(-0.2);
    Distance shooterOffsetY = Meters.of(0);
    Transform3d shooterOffset = new Transform3d(shooterOffsetX, shooterOffsetY,shooterHeight, new Rotation3d(0,0,0));

    Time simTimestep = Milliseconds.of(20);

    //tried in vain to get an algebraic solution for shot speed and especially shot angle - turns out the answer is either empirical or use numerical methods instead of algebraic.
    //this lets you put as many ordered pairs as you want and then it will give a linear interpolation between them for any input
    InterpolatingDoubleTreeMap distance_to_speed = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap distance_to_azimuth = new InterpolatingDoubleTreeMap();
    
    Supplier<Pose2d> poseSupplier;
    Supplier<ChassisSpeeds> speedsSupplier;
    Supplier<Translation3d> targetSupplier;

    Pose2d robotPose;
    Pose3d shooterPose;

    Translation3d target;
    Translation3d robotToTarget;
    double targetDistance;

    Translation3d virtualTarget;
    Translation3d robotToVirtualTarget;
    double virtualTargetDistance;

    LinearVelocity virtualShotVelocity;
    Angle virtualAzimuth;

    LinearVelocity driveVelX;
    LinearVelocity driveVelY;
    
    StructArrayPublisher<Pose3d> trajectoryPublisher;
    DoublePublisher shotVelocityPublisher;
    DoublePublisher shotAnglePublisher;
    DoublePublisher hubDistancePublisher;
    StructPublisher<Translation3d> virtualHubPublisher;

    
    public ShooterSimTest(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier, Supplier<Translation3d> targetSupplier) 
    {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.targetSupplier = targetSupplier;


        //sending data to network tables to visualize in advantagescope
        trajectoryPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Shooter/trajectory",Pose3d.struct).publish();
        shotVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/velocityTarget").publish();
        shotAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/angleTarget").publish();
        hubDistancePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/distanceToHub").publish();
        virtualHubPublisher = NetworkTableInstance.getDefault().getStructTopic("Shooter/virtual hub",Translation3d.struct).publish();
       
       
        //distances to note:
        //1.375 seems to be closest possible shot
        //5.5 is about the longest shot I could see

        //map distance from goal to shot speed
        distance_to_speed.put(0.0,6.0);
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
        distance_to_azimuth.put(8.0,60.0);
        
        
    }

    @Override
    public void periodic() 
    {
        //calculate this continuously so we can tell the drivetrain what yaw to target

        //positions of robot and hub
        robotPose = poseSupplier.get();
        target = targetSupplier.get();

        //3d position of shooter
        shooterPose = new Pose3d(robotPose).plus(shooterOffset);

        
        //points from shooter to target
        robotToTarget = shooterPose.getTranslation().minus(target);

        //vectorTowardsGoal = robotToTargetVector.toTranslation2d().toVector().unit();
        //vectorSidewaysGoal = VecBuilder.fill(-1*vectorTowardsGoal.get(1), vectorTowardsGoal.get(0));
        //distance from shooter to target
        targetDistance = Math.sqrt(Math.pow(robotToTarget.getX(),2) + Math.pow(robotToTarget.getY(),2));
        
        //intended velocity and azimuth (pitch) from lookup table based on distance
        shotVelocity = MetersPerSecond.of(distance_to_speed.get(targetDistance));
        azimuth = Degrees.of(distance_to_azimuth.get(targetDistance));

        double a = 0.5 * gravity.in(MetersPerSecondPerSecond);
        double b = shotVelocity.in(MetersPerSecond)*Math.sin(azimuth.in(Radians));
        double c = shooterHeight.in(Meters) - hub_z;

        double shotTime = (-1*b - Math.sqrt(b*b-4*a*c)) / (2*a);

        SmartDashboard.putNumber("shot time", shotTime);
        driveVelX = MetersPerSecond.of(speedsSupplier.get().vxMetersPerSecond);
        driveVelY = MetersPerSecond.of(speedsSupplier.get().vyMetersPerSecond);
        
        virtualTarget = new Translation3d (
            target.getX()-shotTime*driveVelX.in(MetersPerSecond),
            target.getY()-shotTime*driveVelY.in(MetersPerSecond),
            target.getZ()
        );

        robotToVirtualTarget = shooterPose.getTranslation().minus(virtualTarget);
        virtualTargetDistance = Math.sqrt(Math.pow(robotToVirtualTarget.getX(),2) + Math.pow(robotToVirtualTarget.getY(),2));
        virtualShotVelocity = MetersPerSecond.of(distance_to_speed.get(virtualTargetDistance));
        virtualAzimuth = Degrees.of(distance_to_azimuth.get(virtualTargetDistance));
        

        virtualHubPublisher.set(virtualTarget);
    }

    public Rotation2d targetYaw()
    {
        return new Rotation2d(robotToVirtualTarget.getX(), robotToVirtualTarget.getY()).plus(Rotation2d.k180deg);
    } 

    public Command fire()
    {
        //Simulate a shot
        return runOnce(() -> 
            {
                //list of locations the shot travels through, sent to NetworkTables to visualize
                List<Pose3d> poses = new ArrayList<Pose3d>();
                poses.add(shooterPose);

                //change in shot position each 20ms timestep
                Distance deltaX, deltaY, deltaZ;

                //current shot position
                Distance curX, curY, curZ;
                
                //previous shot position - start it off at the robot's position
                Distance prevX = shooterPose.getMeasureX();
                Distance prevY = shooterPose.getMeasureY();
                Distance prevZ = shooterPose.getMeasureZ();
                
                //azimuth is the vertical angle of the shooter. vertical (Z) part of the shot is sin(azimuth), horizontal (X and Y) part is cos(azimuth)
                double azimuthCos = Math.cos(virtualAzimuth.in(Radians));
                double azimuthSin = Math.sin(virtualAzimuth.in(Radians));

                //yaw is the ground angle of the robot. X part of the shot is cos(yaw), Y part is sin(yaw)
                double yawCos = Math.cos(robotPose.getRotation().getRadians());
                double yawSin = Math.sin(robotPose.getRotation().getRadians());

                
                //robot's drive velocity when shot is taken
                /*Vector<N2> driveVelVector = VecBuilder.fill(speedsSupplier.get().vxMetersPerSecond,
                                                            speedsSupplier.get().vyMetersPerSecond);

                double velTowardsGoal = driveVelVector.projection(vectorTowardsGoal).norm();
                double velSidewaysGoal = driveVelVector.projection(vectorSidewaysGoal).norm();*/
                

                
                
                
                shotVelocityPublisher.set(virtualShotVelocity.in(MetersPerSecond));
                shotAnglePublisher.set(virtualAzimuth.in(Degrees));
                hubDistancePublisher.set(virtualTargetDistance);

                //draw the shot
                for(int i = 1; i < 150; i++)
                {
                    //dx = v_x * dt + v_drive_x * dt
                    deltaX = virtualShotVelocity.times(simTimestep).times(azimuthCos).times(yawCos)
                            .plus(driveVelX.times(simTimestep));
                    //dy = v_y * dt + v_drive_x * dt
                    deltaY = virtualShotVelocity.times(simTimestep).times(azimuthCos).times(yawSin)
                            .plus(driveVelY.times(simTimestep));
                    //dz = v_z * dt + dv_gravity = v_z * dt + g * t * dt
                    deltaZ = virtualShotVelocity.times(simTimestep).times(azimuthSin)
                            .plus(gravity.times(simTimestep).times(simTimestep).times(i));

                    curX = prevX.plus(deltaX);
                    curY = prevY.plus(deltaY);
                    curZ = prevZ.plus(deltaZ);

                    prevX = curX;
                    prevY = curY;
                    prevZ = curZ;

                    poses.add(
                        new Pose3d(
                            new Translation3d(
                                curX,
                                curY,
                                curZ),
                            new Rotation3d(0,0,0)
                        )
                    );
                    
                }


                
                trajectoryPublisher.set(poses.toArray(new Pose3d[0]));
            });
    }
}