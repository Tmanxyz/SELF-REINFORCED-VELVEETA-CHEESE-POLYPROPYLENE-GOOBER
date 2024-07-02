package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings.Shooter;

public class ShooterStop extends ShooterSetRPM {

    public ShooterStop() {
        super(Shooter.PODIUM_SHOT);
    }
}