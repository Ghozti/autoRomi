// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.RomiGyro;

/** Add your docs here. */
public class Forward implements Autonomous{

    RomiGyro gyro;
    RomiDrivetrain drivetrain;

    public Forward(RomiGyro gyro, RomiDrivetrain drivetrain){
        this.gyro = gyro;
        this.drivetrain = drivetrain;
    }

    //PID for auto
    double kP = 0.2;
    double kI = 0.02;
    double kD = 0.02;
    double iLimit = .5;

    //auto stuff
    double distanceTargetInches = 40;
    double error = 0;
    double errorSum = 0;
    double lastError = 0;
    double errorRate = 0;
    double speedOutput = 0;
    double lastTimestamp = 0;
    double currentTime = 0;

    //gyro pid stuff

    double gP = .01;
    double gI = .0001;
    double gD;
    double gilimit = 30;

    double output;
    double gerror;
    double maxTolerance = .8;
    double minTolerance = -.8;
    double gErrorSum;    

    @Override
    public void init() {
        
    }

    @Override
    public void update() {
        /*
    error = distanceTargetInches - ((getLeftDistanceInch() + getRightDistanceInch())/2);
    currentTime = Timer.getFPGATimestamp() - lastTimestamp;

    if(Math.abs(error) < iLimit){
      errorSum += error + currentTime;
    }

    errorRate = (error - lastError)/currentTime;

    speedOutput = (kP * error) + (kI * errorSum) + (kD * errorRate);
    arcadeDrive(speedOutput, 0);

    //update time stamp & error
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;
    */

    currentTime = Timer.getFPGATimestamp() - lastTimestamp;

    if(gyro.getAngleZ() > maxTolerance){
      gerror = maxTolerance - gyro.getAngleZ();
    }
    if(gyro.getAngleZ() < minTolerance){
      gerror = minTolerance - gyro.getAngleZ();
    }

    if(Math.abs(gerror) < gilimit && Math.round(gerror) != 0){
      gErrorSum += gerror + currentTime;
    }

    output = (gP* gerror) + (gI * gErrorSum);

    drivetrain.arcadeDrive(.35, output);

    lastTimestamp = Timer.getFPGATimestamp();
        
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        errorSum = 0;
        lastError = 0;
        lastTimestamp = Timer.getFPGATimestamp();
    }
    
}
