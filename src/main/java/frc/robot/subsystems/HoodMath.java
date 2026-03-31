package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TurretConstants;

public class HoodMath extends SubsystemBase { // the best file name in existence
    private static final InterpolatingDoubleTreeMap kDistanceToAngle = new InterpolatingDoubleTreeMap();
    private NetworkTable limTable;
    private NetworkTableEntry ty;

    double distanceToAprilTag = 0.0;

    private final double ANGLE_LOWER_BOUND = 12.0; // not detected from close up
    private final double ANGLE_UPPER_BOUND = 24.375; // when apriltag not detected from afar

    public enum FieldZone {
        TRENCH,
        NORMAL
    }

    public HoodMath() {
        limTable = NetworkTableInstance.getDefault().getTable("limelight");
        ty = limTable.getEntry("ty");

        kDistanceToAngle.put(1.0, ANGLE_LOWER_BOUND);
        kDistanceToAngle.put(1.15, 14.5);
        kDistanceToAngle.put(1.3, 15.9);
        kDistanceToAngle.put(1.5, 16.35);
        kDistanceToAngle.put(1.75, 17.1);
        kDistanceToAngle.put(2.0, ANGLE_UPPER_BOUND); // 2.3 - 8.0 -> 1.0 - 2.0
        kDistanceToAngle.put(3.3,39.9);
        // kDistanceToAngle.put(0,0);
        SmartDashboard.putNumber("Target Distance", 0);
    }

    public void periodic() {
        ty = limTable.getEntry("ty");
        // distanceToAprilTag = (Constants.targetToGround - Constants.limelightToGround)
        // / Math.tan(ty.getDouble(0)*(Math.PI/180.0));
        double[] target = LimelightHelpers.getBotPose_TargetSpace("limelight");
        double xOffset = target[0];
        double zDistance = target[2];
        distanceToAprilTag = Math.sqrt(Math.pow(xOffset, 2) + Math.pow(zDistance, 2));
    }

    public boolean isAprilTagDetected() {
        return limTable.getEntry("tv").getDouble(0) == 1; // true if detected
    }

    public double getDistanceFromAprilTag() {
        return distanceToAprilTag;
    }

    public FieldZone getZone(Pose2d robotPose) {
        double x = robotPose.getX();
        double y = robotPose.getY();

        return FieldZone.NORMAL; // to be edited later
    }

    public double getHoodAngle(double dist) {
        if (!isAprilTagDetected()) {
            if (Math.abs(distanceToAprilTag) < 1.3) {
                return ANGLE_LOWER_BOUND;
            } else {
                return ANGLE_UPPER_BOUND;
            }
        }
        return kDistanceToAngle.get(dist);
    }
}
