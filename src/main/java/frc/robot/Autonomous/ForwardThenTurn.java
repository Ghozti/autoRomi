// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.RomiGyro;

/** Add your docs here. */
public class ForwardThenTurn implements Autonomous{

    RomiDrivetrain drivetrain;
    RomiGyro gyro;

    public ForwardThenTurn(RomiDrivetrain drivetrain, RomiGyro gyro){
        this.drivetrain = drivetrain;
        this.gyro = gyro;
    }

    double kP = 0.05;
    double kI = 0.02;
    double kD = 0.02;
    double iLimit = .5;

    //gyro
    double gP = .01;
    double gI = .0001;
    double gD;
    double gilimit = 30;
    double gerror = 0;
    double gerrorSum;

    double distanceTargetInches = 10;
    double turnDegrees = 90;
    double error = 0;
    double errorSum = 0;
    double lastError = 0;
    double errorRate = 0;
    double speedOutput = 0;
    double lastTimestamp = 0;
    double currentTime = 0;
    double output;

    @Override
    public void init() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub

        error = distanceTargetInches - ((drivetrain.getLeftDistanceInch() + drivetrain.getRightDistanceInch())/2);
        currentTime = Timer.getFPGATimestamp() - lastTimestamp;

        if(Math.abs(error) < iLimit){
            errorSum += error + currentTime;
        }

        errorRate = (error - lastError)/currentTime;

        speedOutput = (kP*error) + (kI * errorSum) + (kD * errorRate);
        drivetrain.arcadeDrive(speedOutput, 0);

        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
        
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        errorSum = 0;
        lastError = 0;
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

}
