package org.usfirst.frc.team5507.robot.commands;

import org.usfirst.frc.team5507.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class DriveSidewaysDistance extends Command {
  private double d1; 
  private double startAngle;
  private double speed;
  public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
  public static final double TOTAL_SENSOR_POS = 1024;
  public static final double DISTANCE = WHEEL_CIRCUMFERENCE / TOTAL_SENSOR_POS;

  public DriveSidewaysDistance(double d, double speed) {
    requires(Robot.swerveDriveSubsystem);
    d1 = DISTANCE * d * 12;
    this.speed = speed;
  }

  @Override
  protected void initialize() {
    startAngle = Robot.swerveDriveSubsystem.getNavX().getYaw();
  }

  @Override
  protected void execute() {
    Robot.swerveDriveSubsystem.driveForwardDistance(d1, startAngle);
  }

  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.swerveDriveSubsystem.calculateErrPos(d1)) < DISTANCE;
     
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
