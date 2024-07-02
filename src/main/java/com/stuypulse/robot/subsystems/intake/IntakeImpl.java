package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeImpl extends Intake {

    private final CANSparkMax motor;
    private final CANSparkMax leftFunnelMotor;
    private final CANSparkMax rightFunnelMotor;


    private final BStream stalling;

    protected IntakeImpl() {
        motor = new CANSparkMax(Ports.Intake.MOTOR, MotorType.kBrushless);
        leftFunnelMotor = new CANSparkMax(Ports.Intake.MOTOR, MotorType.kBrushless);
        rightFunnelMotor = new CANSparkMax(Ports.Intake.MOTOR, MotorType.kBrushless);

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounceRC.Rising(Settings.Intake.Detection.STALL_TIME));

        Motors.Intake.MOTOR_CONFIG.configure(motor);
        Motors.Intake.LEFT_MOTOR_CONFIG.configure(leftFunnelMotor);
        Motors.Intake.RIGHT_MOTOR_CONFIG.configure(rightFunnelMotor);
    }

    @Override
    public void acquire() {
        motor.set(+Settings.Intake.ACQUIRE_SPEED);
        rightFunnelMotor.set(+Settings.Intake.ACQUIRE_SPEED);
        leftFunnelMotor.set(+Settings.Intake.ACQUIRE_SPEED);
    }

    @Override
    public void deacquire() {
        motor.set(-Settings.Intake.DEACQUIRE_SPEED);
        rightFunnelMotor.set(-Settings.Intake.DEACQUIRE_SPEED);
        leftFunnelMotor.set(-Settings.Intake.DEACQUIRE_SPEED);
    }

    @Override
    public void stop() {
        motor.stopMotor();
        rightFunnelMotor.stopMotor();
        leftFunnelMotor.stopMotor();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        motor.setIdleMode(mode);
        rightFunnelMotor.setIdleMode(mode);
        leftFunnelMotor.setIdleMode(mode);

        motor.burnFlash();
        rightFunnelMotor.burnFlash();
        leftFunnelMotor.burnFlash();
    }

    // Detection

    private boolean isMomentarilyStalling() {
        return motor.getOutputCurrent() > Settings.Intake.Detection.STALL_CURRENT;
    }

    private boolean isStalling() {
        return stalling.get();
    }

    @Override
    public double getIntakeRollerSpeed() {
        return motor.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake/Speed", motor.get());
        SmartDashboard.putNumber("Intake/Current", motor.getOutputCurrent());
        
        SmartDashboard.putNumber("Intake/Left Speed", leftFunnelMotor.get());
        SmartDashboard.putNumber("Intake/Left Current", leftFunnelMotor.getOutputCurrent());

        SmartDashboard.putNumber("Intake/Right Speed", rightFunnelMotor.get());
        SmartDashboard.putNumber("Intake/Right Current", rightFunnelMotor.getOutputCurrent());

        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());
        SmartDashboard.putBoolean("Intake/Above Current Limit", isMomentarilyStalling());
    }
}