package org.firstinspires.ftc.teamcode.seasons.relicrecovery;

import org.firstinspires.ftc.teamcode.algorithms.IGyroPivotAlgorithm;
import org.firstinspires.ftc.teamcode.algorithms.impl.DistanceSensorDriveAlgorithm;
import org.firstinspires.ftc.teamcode.mechanism.impl.BNO055IMUWrapper;
import org.firstinspires.ftc.teamcode.seasons.relicrecovery.algorithms.impl.VuMarkScanAlgorithm;

public class AutonomousColorPointOfReference
{
    private RelicRecoveryRobot robot;
    private VuMarkScanAlgorithm vuMarkScanAlgorithm;
    private IGyroPivotAlgorithm gyroPivotAlgorithm;
    private BNO055IMUWrapper bno055IMUWrapper;

    private DistanceSensorDriveAlgorithm rightDistanceSensorDrive;
    private DistanceSensorDriveAlgorithm leftDistanceSensorDrive;

//    public void detectColorOfStone();
//    public void vuMarkScan();
//    public void driveIntoCryptobox();
//    public void knockJewel();
//    public void driveBalancingStone();
//    public void driveToCryptoBox();
}

//new AutonomousColorPointOfReference("Blue", "Back");
//new AutonomousColorPointOfReference("Blue", "Front");