package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.utils.CoordinateSpace;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.math.*;
import java.util.ArrayList;
import java.util.List;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.path.PathPlannerPath;

public class Eyes extends SubsystemBase{
    private static Eyes instance;
    public final RobotContainer roboContainer = RobotContainer.getInstance();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    

    //NetworkTableEntry tagID = table.getentry
    // NetworkTableEntry tx;
    // NetworkTableEntry ty;
    // NetworkTableEntry ta;
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);
    // I'm pretty sure these values need to be in periodic or has to be updated a lot. - seo
    
    public void aligningRobot(double[] xAndY){
        double y = xAndY[1];
        double goalRadians=(Constants.VisionConstants.mountAngle+y) * (3.14159 / 180);
        //From what I know getTX gets the targets x value then return a degree value based on the horizontal fov/2 and -fov/2 - seo
        double tagAngle;//LimelightHelpers.getTX("limelight"); same as line below
        tagAngle = xAndY[0];
        double tagID = LimelightHelpers.getFiducialID("limelight");
        //probaly need to set a priority of tragets such as amp or human player.
        double distanceToGoal;
        //there is a visual on how this works. With i think both the angle and distance we can try a drive command -seo
        //The height of the april tag can be in a in dfferent heights so for each id we will calculate distance with different heights.
        switch ((int)tagID){
            case 1:
                distanceToGoal= (Constants.VisionConstants.ampHeight-Constants.VisionConstants.lensHeight)/Math.tan(goalRadians);
                //probably add a drive function
                List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    new Pose2d(roboContainer.m_robotDrive.getPose().getX(), roboContainer.m_robotDrive.getPose().getY(), Rotation2d.fromDegrees(0)),
                    new Pose2d()
                    )
                
                
                break;
            case 2:
                distanceToGoal = (Constants.VisionConstants.humanHeight-Constants.VisionConstants.lensHeight)/Math.tan(goalRadians);
                break;
            default:
                distanceToGoal = (Constants.VisionConstants.goalHeight-Constants.VisionConstants.lensHeight)/Math.tan(goalRadians);
                break;
        }
        


    }

    public ArrayList<Double> collectValues(){
        ArrayList<Double> allValues = new ArrayList<>();
        for(int pipeNum=0;pipeNum<=9;pipeNum++){
            //might be the same as LimelightHelpers.setPipelineIndex("Limelightname", "the index");
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeNum);
            //table.getEntry("Value").getDouble() might be the same as the LimelightHelpers.GetTX("name")
            allValues.add(table.getEntry("tx").getDouble(0.0));
            allValues.add(table.getEntry("ty").getDouble(0.0));
        }
        return allValues;

    }

    public double[] averageData(ArrayList<Double> data){
        double[] average ={0.0,0.0};
        int xCount = 0;
        double totalX = 0.0;
        int yCount = 0;
        double totalY = 0.0;
        for(int intervalX = 0; intervalX<=9; intervalX+=2){
            if(data.get(intervalX*2)!= 0.0){
                xCount++;
                totalX+=data.get(intervalX);
            }
        }
        
        for(int intervalY = 1; intervalY<=9; intervalY+=2){
            if(data.get(intervalY)!= 0.0){
                xCount++;
                totalX+=data.get(intervalY);
            }
        }
        average[0]=totalX/xCount;
        average[1]=totalY/yCount;
        return average;
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("tx ", x);
        ArrayList<Double> data = collectValues();
        double[] bothData = averageData(data);
        // x = table1.getEntry("tx").getDouble(0.0);
        // y = table1.getEntry("ty").getDouble(0.0);
        // area = table1.getEntry("ta").getDouble(0.0);


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
    public static Eyes getInstance(){
        if(instance == null) instance = new Eyes();
        return instance;
    }
}