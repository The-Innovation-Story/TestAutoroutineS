// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax FR;
  private final CANSparkMax BR;
  public RelativeEncoder R_encoder;
  private final MotorControllerGroup rightSide;

  private final CANSparkMax FL;
  private final CANSparkMax BL;
  public RelativeEncoder L_encoder;
  private final MotorControllerGroup leftSide;

  private final DifferentialDrive driveTrain;

  // The right-side drive encoder
  // public RelativeEncoder m_rightEncoder =
  // new Encoder(
  // DriveConstants.kRightEncoderPorts[0],
  // DriveConstants.kRightEncoderPorts[1],
  // DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  public Pose2d pose1;
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    this.FR = new CANSparkMax(12, MotorType.kBrushless);
    this.BR = new CANSparkMax(11, MotorType.kBrushless);
    this.R_encoder = this.FR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DriveConstants.neoCountsPerRevolution);
    this.rightSide = new MotorControllerGroup(this.FR, this.BR);

    this.FL = new CANSparkMax(22, MotorType.kBrushless);
    this.BL = new CANSparkMax(21, MotorType.kBrushless);
    this.L_encoder = this.FL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DriveConstants.neoCountsPerRevolution);
    this.leftSide = new MotorControllerGroup(this.FL, this.BL);

    this.driveTrain = new DifferentialDrive(this.leftSide, this.rightSide);
    // Pose2d pose1;
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // this.rightSide.setInverted(true);
    // this.leftSide.setInverted(false);

    // Sets the distance per pulse for the encoders
    // L_encoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    // R_encoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    FR.setInverted(true);
    BR.setInverted(true);
    FL.setInverted(false);
    BL.setInverted(false);
    // m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    m_odometry = new DifferentialDriveOdometry(getHeading());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    pose1 = m_odometry.update(getHeading(),
        L_encoder.getPosition() * (Math.PI * Units.inchesToMeters(6)) / 7.31,
        R_encoder.getPosition() * (Math.PI * Units.inchesToMeters(6)) / 7.31);
    // Pose2d pose1=m_odometry.update(m_gyro.getRotation2d(),
    // L_encoder.getPosition() * DriveConstants.converter_distance_from_rotations,
    // R_encoder.getPosition() * DriveConstants.converter_distance_from_rotations);
    SmartDashboard.putNumber("xrc", getPose().getX());
    SmartDashboard.putNumber("yrc", getPose().getY());
    SmartDashboard.putNumber("anglerc", getPose().getRotation().getDegrees());
    // SmartDashboard.putNumber("angle", pose1.getA);
    SmartDashboard.putNumber("angle", m_gyro.getAngle());
    SmartDashboard.putNumber("leftMetersPerSecond", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("rightMetersPerSecond", getWheelSpeeds().rightMetersPerSecond);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return pose1;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(L_encoder.getVelocity() * DriveConstants.rpm_to_ms_wheel_converter / 7.31,
        R_encoder.getVelocity() * DriveConstants.rpm_to_ms_wheel_converter / 7.31);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    this.driveTrain.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    this.leftSide.setVoltage(leftVolts);
    this.rightSide.setVoltage(rightVolts);
    // this.leftSide.setVoltage(rightVolts);
    // this.rightSide.setVoltage(leftVolts);
    this.driveTrain.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    L_encoder.setPosition(0.0);
    R_encoder.setPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // return (L_encoder.getPosition() + R_encoder.getPosition()) / 2;
    return (L_encoder.getPosition() + R_encoder.getPosition()) * DriveConstants.neoMotorWheelCircumference
        * DriveConstants.motorGearBoxRatio / (2.0 * DriveConstants.neoEncoderCountsPerRev);
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return L_encoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return R_encoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    this.driveTrain.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  // public double getHeading() {
  // // double ypr[] = {0,0,0};
  // // m_gyro.getAngleAdjustment();
  // // return Math.IEEEremainder(ypr[0], 360.0d);
  // return m_gyro.getRotation2d().getDegrees();
  // }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());

  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public double getDistanceTravelled() {
    double totalDistance = ((Math.abs(L_encoder.getPosition()) + Math.abs(R_encoder.getPosition()))
        / DriveConstants.neoEncoderCountsPerRev) * DriveConstants.neoMotorWheelCircumference
        * DriveConstants.motorGearBoxRatio; // RobotContainer.enc_L.getDistance();
    return ((totalDistance) / 2);
  }
}
