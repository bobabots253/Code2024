package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;
import frc.robot.States;
import frc.utils.Util;

public class Hook extends SubsystemBase {
    private static final CANSparkMax m_Hook = Util.createSparkMAX(HookConstants.motorID, MotorType.kBrushless);
    private SparkAbsoluteEncoder armEncoder = m_Hook.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder relArmEncoder = m_Hook.getEncoder();

    private SparkPIDController pidController;
    
    
    private static Hook instance;
    public static Hook getInstance() {
        if(instance == null) instance = new Hook();
        return instance;

    }
    
    
    private Hook() {
    
        resetEncoders();
       
        m_Hook.setInverted(false);
       
        m_Hook.setIdleMode(IdleMode.kBrake);

        pidController = m_Hook.getPIDController();
        pidController.setP(HookConstants.kP); //0.1
        pidController.setI(HookConstants.kI);//0.0
        pidController.setD(HookConstants.kD);//0.05
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(-0.3, 0.3);
        register();
    }



    public void setOpenLoop(double value) {
        SmartDashboard.putNumber("Arm Commanded arm actuation", value);
        m_Hook.set(value);
        
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
    }
    


    public void setHookPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("Arm SetPoint", position);
    }

    public void setHookPositionDegree(double degreePosition) { //define degreePosition earlier
        double convertDeg = 11.375;
        double encoderPosition = degreePosition*convertDeg; //degree to encoder
        double currentPosition = armEncoder.getPosition(); 
        SmartDashboard.putNumber("degreePosition", degreePosition);
        SmartDashboard.putNumber("encoder value", encoderPosition);
        SmartDashboard.putNumber("current arm position", currentPosition);
    }

    public void setHookState(States.HookPos state) {
        switch (state) {
            case STOW:
                setHookPosition(HookConstants.stow);
                break;
            case OPEN:
                setHookPosition(HookConstants.open);
                break;
            default:
                setHookPosition(HookConstants.score);
                break;
            //default to STOW?
        }
    }
}