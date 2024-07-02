/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    double DT = 1.0 / 50.0;

    double WIDTH = Units.inchesToMeters(32);
    double LENGTH = Units.inchesToMeters(36);
    
    public interface Intake {
        public interface Detection {
            double STALL_TIME = 0.05;
            double STALL_CURRENT = 50;
        }

        double ACQUIRE_SPEED = 1.0;
        double DEACQUIRE_SPEED = 1.0;
    }
    
    public interface Shooter {
        double SHOOT_TIME_DEBOUNCE = 0.4;

        double MOMENT_OF_INERTIA = 0.01;

        double TELEOP_SHOOTER_STARTUP_DELAY = 0.5;

        // MAX RPM
        // LEFT/RIGHT: 5900
        // FEEDER: 3100
        ShooterSpeeds PODIUM_SHOT = new ShooterSpeeds(
            new SmartNumber("Shooter/Podium Shooter RPM", 5500),
            //500 differential,
            new SmartNumber("Shooter/Podium Feeder RPM", 3000));
        
        ShooterSpeeds AMP_SCORE = new ShooterSpeeds(-3000, -3000);

        ShooterSpeeds FERRY = new ShooterSpeeds(
            new SmartNumber("Shooter/Ferry Shooter RPM", 6000), // 5500
            //500,
            new SmartNumber("Shooter/Ferry Feeder RPM", 3500)); // 3000

        ShooterSpeeds WING_FERRY = new ShooterSpeeds(2000, 2500);

        double AT_RPM_EPSILON = 200;

        SmartNumber RPM_CHANGE_RC = new SmartNumber("Shooter/RPM Change RC", 0.2);
        double RPM_CHANGE_DIP_THRESHOLD = 300;

        public interface Feedforward {
            double kS = 0.11873;
            double kV = 0.0017968;
            double kA = 0.00024169;
        }

        public interface PID {
            double kP = 0.00034711;
            double kI = 0;
            double kD = 0.0;
        }
    }

    public interface Feeder {
        double GEARING = 18.0 / 30.0;
        double POSITION_CONVERSION = GEARING;
        double VELOCITY_CONVERSION = POSITION_CONVERSION / 60;

        public interface Feedforward {
            double kS = 0.71611;
            double kV = 0.00335;
            double kA = 0.076981;
        }

        public interface PID {
            double kP = 0.00020863;
            double kI = 0.0;
            double kD = 0.0;
        }
    }
        public interface Swerve {
        // between wheel centers
        double WIDTH = Units.inchesToMeters(20.75);
        double LENGTH = Units.inchesToMeters(20.75);
        double CENTER_TO_INTAKE_FRONT = Units.inchesToMeters(13.0);

        double MAX_MODULE_SPEED = 4.9;
        double MAX_MODULE_ACCEL = 15.0;

        double MODULE_VELOCITY_DEADBAND = 0.05;

        SmartNumber ALIGN_OMEGA_DEADBAND = new SmartNumber("Swerve/Align Omega Deadband", 0.05); // TODO: make 0.25 and test

        public interface Assist {
            SmartNumber ALIGN_MIN_SPEAKER_DIST = new SmartNumber("SwerveAssist/Minimum Distance to Speaker", 4); //change

            double BUZZ_INTENSITY = 1;

            // angle PID
            SmartNumber kP = new SmartNumber("SwerveAssist/kP", 6.0);
            SmartNumber kI = new SmartNumber("SwerveAssist/kI", 0.0);
            SmartNumber kD = new SmartNumber("SwerveAssist/kD", 0.0);

            double ANGLE_DERIV_RC = 0.05;
            double REDUCED_FF_DIST = 0.75;
        }

        // TODO: Tune these values
        public interface Motion {
            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity", 3.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration", 4.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity", Units.degreesToRadians(540));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration", Units.degreesToRadians(720));

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());

            PIDConstants XY = new PIDConstants(2.5, 0, 0.02);
            PIDConstants THETA = new PIDConstants(4, 0, 0.1);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 6.12;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }

        public interface Turn {
            double kP = 7.0;
            double kI = 0.0;
            double kD = 0.05;

            double kS = 0.25582;
            double kV = 0.00205;
            double kA = 0.00020123;
        }

        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/PID/kP", 0.31399);
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.27354;
            SmartNumber kV = new SmartNumber("Swerve/Drive/FF/kV", 2.1022);
            SmartNumber kA = new SmartNumber("Swerve/Drive/FF/kA", 0.41251);
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(38.144531);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-173.408203);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(24.609375);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(38.232422);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }
    }
}
