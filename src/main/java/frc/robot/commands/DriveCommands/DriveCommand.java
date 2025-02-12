package frc.robot.commands.DriveCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drive.Swerve;
import edu.wpi.first.math.controller.PIDController;

public class DriveCommand extends Command{
    Swerve swerve;
    Supplier<Double> xSpeed, ySpeed, zSpeed;
    private final PIDController xPID, yPID, zPID;
    private boolean robotRelative = false;
    private double chassisMaxOutput;

    public DriveCommand(Swerve swerve, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> zSpeed, 
                        boolean robotRelative, double chassisMaxOutput){
        addRequirements(swerve);
        this.swerve =swerve;
        
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;
        this.robotRelative = robotRelative;
        this.chassisMaxOutput = chassisMaxOutput;
        
        this.xPID = new PIDController(SwerveConstants.chassisXYAccelerationkP, 0, 0);
        this.yPID = new PIDController(SwerveConstants.chassisXYAccelerationkP, 0, 0);
        this.zPID = new PIDController(SwerveConstants.chassisZAccelerationkP, 0, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xNeed = this.xSpeed.get();
        double yNeed = this.ySpeed.get();
        double zNeed = this.zSpeed.get();

        // 2. Apply deadband
        xNeed = Math.abs(xNeed) > SwerveConstants.kDeadband ? xNeed : 0.0;
        yNeed = Math.abs(yNeed) > SwerveConstants.kDeadband ? yNeed : 0.0;
        zNeed = Math.abs(zNeed) > SwerveConstants.kDeadband ? zNeed : 0.0;

        // 3. Make the driving smoother
        xNeed = xPID.calculate(xNeed);
        yNeed = yPID.calculate(yNeed);
        zNeed = zPID.calculate(zNeed);
        
        swerve.setChassisSpeeds(xNeed, yNeed, zNeed, robotRelative, chassisMaxOutput);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}