package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Module extends SubsystemBase{
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private double realAngle = 0;
    private double desiredAngle = 0;
    private double PIDvalue = 0;

    private final CANcoder turnEncoder;
    private final PIDController turnPID;

    private final double offset;
    public String moduleName;
    private double turnMotorvalue;

    public Module(int driveMotorID, int turnMotorID, int CANcoderID, double kP, double kI, double kD , double turnEncoderOffset, String moduleName) {
        this.offset = turnEncoderOffset;

        this.driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        this.turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
        
        this.turnEncoder = new CANcoder(CANcoderID, "Drivetrain");
        this.turnPID = new PIDController(kP, kI, kD);

        this.turnPID.enableContinuousInput(-180, 180);

        this.moduleName = moduleName;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getAngle());
      }

    public void setDesiredState(SwerveModuleState desiredState, String moduleName){
        desiredState =  SwerveModuleState.optimize(desiredState, getAngle()); 

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.05){
            stop();
            return;
        }
        setSpeed(desiredState, moduleName);
        setAngle(desiredState, moduleName);
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void setSpeed(SwerveModuleState desiredState, String moduleName){
        turnMotorvalue = desiredState.speedMetersPerSecond; 
        driveMotor.set(turnMotorvalue);
    }

    public void setAngle(SwerveModuleState desiredState, String moduleName){
        realAngle =  getAngle().getDegrees();
        desiredAngle = desiredState.angle.getDegrees();
        PIDvalue = turnPID.calculate(realAngle, desiredAngle);
        turnMotor.set(PIDvalue);
    }
    
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(getTurnEncoder());
    }

    public double getTurnEncoder(){
        return turnEncoder.getAbsolutePosition().getValueAsDouble() * -360 + this.offset;
        // return turnEncoder.getAbsolutePosition().getValue() * -360 + this.offset;
    }

    public SwerveModuleState getSwerveState(){
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

     public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity() * SwerveConstants.driveRPS2MPS;
    }

    public double getDrivePosition(){
        double position;
        position = driveMotor.getEncoder().getPosition();
        position *= SwerveConstants.driveRevsToMeters;
        return position;
    }

    public double getDriveEncoderVelocity(){
        return driveMotor.getEncoder().getVelocity();
    }

    @Override
    public void periodic(){
    }
}