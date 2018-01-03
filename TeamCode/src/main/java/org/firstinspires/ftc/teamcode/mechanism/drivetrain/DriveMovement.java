package org.firstinspires.ftc.teamcode.mechanism.drivetrain;

/**
 *
 */

public class DriveMovement {
    private int targetDistance;
    private double speed;
    private double angleDegrees;
    private boolean nonBlocking;

    public DriveMovement(int targetDistance, double speed, double angleDegrees, boolean nonBlocking) {
        this.targetDistance = targetDistance;
        this.speed = speed;
        this.angleDegrees = angleDegrees;
        this.nonBlocking = nonBlocking;
    }

    public int getTargetDistance() {
        return targetDistance;
    }

    public double getSpeed() {
        return speed;
    }

    public double getAngleDegrees() {
        return angleDegrees;
    }

    public boolean isNonBlocking() {
        return nonBlocking;
    }
}
