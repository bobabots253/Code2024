package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States;
import frc.robot.Constants.ArmConstants;
import frc.utils.Util;


public class Arm extends SubsystemBase {
    private static final CANSparkMax motorR = Util.createSparkMAX(ArmConstants.rightArmMotorID, MotorType.kBrushless);
    private static final CANSparkMax motorL = Util.createSparkMAX(ArmConstants.leftArmMotorID, MotorType.kBrushless);
    private SparkAbsoluteEncoder armEncoder = motorR.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder relArmEncoder = motorR.getEncoder();

    private SparkPIDController pidController;
    
    
    private static Arm instance;
    public static Arm getInstance() {
        if(instance == null) instance = new Arm();
        return instance;

    }
    
    
    private Arm() {
        resetEncoders();
        motorR.setInverted(false);
        motorL.follow(motorR, true);
       
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setIdleMode(IdleMode.kBrake);
        // armEncoder.setZeroOffset();
        pidController = motorR.getPIDController();
        // pidController.setFeedbackDevice(armEncoder);
        pidController.setP(ArmConstants.kP); //0.1  All values currently set to 0.0
        pidController.setI(ArmConstants.kI);//0.01
        pidController.setD(ArmConstants.kD);
        pidController.setIZone(0);
        // pidController.setFF(0);
        pidController.setOutputRange(ArmConstants.pidOutputLow, ArmConstants.pidOutputHigh);
        register();
    }



    public void setOpenLoop(double value) {
        SmartDashboard.putNumber("Arm Commanded arm actuation", value);
        motorR.set(value);
        
    }
    
    public void stopArm() {
    
        setOpenLoop(0);
    }
    
    /**
     * Resets encoders to zero
     */
    public void resetEncoders() {
        relArmEncoder.setPosition(0.0);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("right Arm abs encoder", (armEncoder).getPosition());
        SmartDashboard.putNumber("right Arm abs encoder degrees", 360.0*armEncoder.getPosition());
        SmartDashboard.putNumber("right Arm Relative encoder value", relArmEncoder.getPosition());
        // if(relArmEncoder.getPosition()>=ArmConstants.max){
        //     stopArm();
        // }

    }
    


    public void setArmPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("Arm SetPoint", position);
    }

    public void setArmState(States.ArmPos state) {
        switch (state) {
            case STOW:
                SmartDashboard.putNumber("Position",0);
                setArmPosition(ArmConstants.stow);
                break;
            default:
                SmartDashboard.putNumber("Position", 1);
                setArmPosition(ArmConstants.score);
                break;
        }
    }

    public void setArmPositionDegree(double degreePosition) { //define degreePosition earlier
        double convertDeg = 11.375;
        double encoderPosition = degreePosition*convertDeg; //degree to encoder
        double currentPosition = armEncoder.getPosition(); 
        SmartDashboard.putNumber("degreePosition", degreePosition);
        SmartDashboard.putNumber("encoder value", encoderPosition);
        SmartDashboard.putNumber("current arm position", currentPosition);
    }
}
