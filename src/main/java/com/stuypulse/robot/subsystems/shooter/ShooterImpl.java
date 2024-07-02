package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.HighPassFilter;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Feeder;
import com.stuypulse.robot.constants.Settings.Shooter.Feedforward;
import com.stuypulse.robot.constants.Settings.Shooter.PID;
import com.stuypulse.robot.util.FilteredRelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ShooterImpl extends Shooter {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax feederMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder feederEncoder;

    private final Controller leftController;
    private final Controller rightController;
    private final Controller feederController;
    
    private final IStream rpmChange;

    protected ShooterImpl() {
        leftMotor = new CANSparkMax(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.Shooter.RIGHT_MOTOR, MotorType.kBrushless);
        feederMotor = new CANSparkMax(Ports.Conveyor.FEEDER, MotorType.kBrushless);

        leftEncoder = new FilteredRelativeEncoder(leftMotor);
        rightEncoder = new FilteredRelativeEncoder(rightMotor);
        feederEncoder = new FilteredRelativeEncoder(feederMotor);

        leftEncoder.setVelocityConversionFactor(1.0);
        rightEncoder.setVelocityConversionFactor(1.0);
        feederEncoder.setVelocityConversionFactor(Feeder.GEARING);

        leftController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD)
                .setIntegratorFilter(1000, 0.5 / PID.kI));
        rightController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD)
                .setIntegratorFilter(1000, 0.5 / PID.kI));
        feederController = new MotorFeedforward(Feeder.Feedforward.kS, Feeder.Feedforward.kV, Feeder.Feedforward.kA).velocity()
            .add(new PIDController(Feeder.PID.kP, Feeder.PID.kI, Feeder.PID.kD)
                .setIntegratorFilter(1000, 1.0 / PID.kI));
        
        rpmChange = IStream.create(this::getAverageShooterRPM)
            .filtered(new HighPassFilter(Settings.Shooter.RPM_CHANGE_RC));

        feederEncoder.setPositionConversionFactor(Feeder.POSITION_CONVERSION);
        feederEncoder.setPositionConversionFactor(Feeder.VELOCITY_CONVERSION);

        Motors.Shooter.LEFT_SHOOTER.configure(leftMotor);
        Motors.Shooter.RIGHT_SHOOTER.configure(rightMotor);
        Motors.Conveyor.SHOOTER_FEEDER_MOTOR.configure(feederMotor);
    }

    @Override
    public double getLeftShooterRPM() {
        return leftEncoder.getVelocity();
    }

    @Override
    public double getRightShooterRPM() {
        return rightEncoder.getVelocity();
    }
    
    @Override
    public double getFeederRPM() {
        return feederEncoder.getVelocity();
    }

    @Override
    public boolean noteShot() {
        return getLeftTargetRPM() > 0 && 
               getRightTargetRPM() > 0 && 
               rpmChange.get() < -Settings.Shooter.RPM_CHANGE_DIP_THRESHOLD;
    }

    @Override
    public void periodic() {
        super.periodic();

        leftController.update(getLeftTargetRPM(), getLeftShooterRPM());
        rightController.update(getRightTargetRPM(), getRightShooterRPM());
        feederController.update(getFeederTargetRPM(), getFeederRPM());

        if (irSensor.get() == false) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
            feederMotor.stopMotor();

            SmartDashboard.putBoolean("Shooter/IRSensor", irSensor.get());
        }

        if (getLeftTargetRPM() == 0 && getRightTargetRPM() == 0 && getFeederTargetRPM() == 0) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
            feederMotor.stopMotor();

            SmartDashboard.putNumber("Shooter/Left Requested Voltage", 0);
            SmartDashboard.putNumber("Shooter/Right Requested Voltage", 0);
            SmartDashboard.putNumber("Shooter/Feeder Requested Voltage", 0);
        } else {
            leftMotor.setVoltage(leftController.getOutput());
            rightMotor.setVoltage(rightController.getOutput());
            feederMotor.setVoltage(feederController.getOutput());
            
            SmartDashboard.putNumber("Shooter/Left Requested Voltage", leftController.getOutput());
            SmartDashboard.putNumber("Shooter/Right Requested Voltage", rightController.getOutput());
            SmartDashboard.putNumber("Shooter/Feeder Requested Voltage", feederController.getOutput());
        }

        SmartDashboard.putNumber("Shooter/Right RPM", getRightShooterRPM());
        SmartDashboard.putNumber("Shooter/Left RPM", getLeftShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM", getFeederRPM());
        
        SmartDashboard.putNumber("Shooter/Feeder Linear Velocity", getFeederRPM() * Units.inchesToMeters(1.0) * Math.PI);
        
        SmartDashboard.putNumber("Shooter/Right Error", rightController.getError());
        SmartDashboard.putNumber("Shooter/Left Error", leftController.getError());
        SmartDashboard.putNumber("Shooter/Feeder Error", feederController.getError());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederMotor.getBusVoltage() * feederMotor.getAppliedOutput());

        SmartDashboard.putNumber("Shooter/Left Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Right Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Feeder Current", feederMotor.getOutputCurrent());

        SmartDashboard.putNumber("Shooter/RPM Change", rpmChange.get());
        SmartDashboard.putBoolean("Shooter/Note Shot", noteShot());
    }
}