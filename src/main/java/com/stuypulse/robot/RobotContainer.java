/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeDeacquire;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.shooter.AmpScore;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final Intake intake = Intake.getInstance();
    public final Shooter shooter = Shooter.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureDriverBindings();
    }

    private void configureDriverBindings() {
        // intaking
        driver.getLeftTriggerButton()
            .whileTrue(new IntakeAcquire());

        driver.getDPadLeft()
            .onTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());

        //shooting
        driver.getRightTriggerButton()
            .onTrue(new ShooterPodiumShot());

        driver.getDPadDown()
            .onTrue(new AmpScore());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
