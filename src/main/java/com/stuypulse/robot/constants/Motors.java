package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

public interface Motors{

    public interface Intake {
        CANSparkConfig MOTOR_CONFIG = new CANSparkConfig(false, IdleMode.kBrake, 500, 0.25);
        CANSparkConfig LEFT_MOTOR_CONFIG =  new CANSparkConfig(false, IdleMode.kBrake, 500, 0.25);
        CANSparkConfig RIGHT_MOTOR_CONFIG = new CANSparkConfig(true, IdleMode.kBrake, 500, 0.25);
    }

    public interface Shooter {
        CANSparkConfig LEFT_SHOOTER = new CANSparkConfig(false, IdleMode.kCoast, 500, 0.5);
        CANSparkConfig RIGHT_SHOOTER = new CANSparkConfig(true, IdleMode.kCoast, 500, 0.5);
    }

    public interface Conveyor {
        CANSparkConfig SHOOTER_FEEDER_MOTOR = new CANSparkConfig(false, IdleMode.kBrake, 500, 0.1);
    }

    public interface Swerve {
        CANSparkConfig DRIVE_CONFIG = new CANSparkConfig(true, IdleMode.kBrake, 60, 0.1);
        CANSparkConfig TURN_CONFIG = new CANSparkConfig(false, IdleMode.kBrake, 80);
    }

    public static class CANSparkConfig {
        public final boolean INVERTED;
        public final IdleMode IDLE_MODE;
        public final int CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public CANSparkConfig(
                boolean inverted,
                IdleMode idleMode,
                int currentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.IDLE_MODE = idleMode;
            this.CURRENT_LIMIT_AMPS = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public CANSparkConfig(boolean inverted, IdleMode idleMode, int currentLimitAmps) {
            this(inverted, idleMode, currentLimitAmps, 0.0);
        }

        public CANSparkConfig(boolean inverted, IdleMode idleMode) {
            this(inverted, idleMode, 500);
        }

        public void configure(CANSparkBase motor) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.burnFlash();
        }
    }
}