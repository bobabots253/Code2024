package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.utils.CoordinateSpace;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.math.*;

public class Eyes {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);

    double area = ta.getDouble(0.0);
    // I'm pretty sure these values need to be in periodic or has to be updated a lot. - seo

    public void aligningRobot(){
        double y = ty.getDouble(0.0);
        double goalRadians = (Constants.VisionConstants.mountAngle+y) * (3.14159 / 180);
        //From what I know getTX gets the targets x value then return a degree value based on the horizontal fov/2 and -fov/2 - seo
        double tagAngle = LimelightHelpers.getTX("limelight");

        double distanceToGoal = (Constants.VisionConstants.goalHeight-Constants.VisionConstants.lensHeight)/Math.tan(goalRadians);
        //there is a visual on how this works. With i think both the angle and distance we can try a drive command -seo


    }

    // 1. Compute direction offset from tag
    public static double widthOffset(AprilTagDetection aprilTag, CoordinateSpace coordinateSpace) {
        double coordinateCenter = coordinateSpace.width/2;
        double aprilTagcenter = aprilTag.getCenterX();
        double offset = aprilTagcenter - coordinateCenter;
        return offset; // / coordinateSpace.width
    }

    // 2. turn offset into movement direction
    public static double rotationRate(double offset)
    {
        double zeroAccuracy = 0.04;
        //double multipliedOffset = 200*offset; 
        SmartDashboard.putNumber("Offset", offset);
        if (offset < zeroAccuracy && offset > -zeroAccuracy) {
            SmartDashboard.putNumber("Centered", 1);
            return 0.0;
            
        } else if (offset < 0) {
            SmartDashboard.putNumber("Going right", 1);
            return 0.1;
        } else {
            SmartDashboard.putNumber("Going left", 1);
            return -0.1;
        }
        
    }
}