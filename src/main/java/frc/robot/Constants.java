// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.61;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot.
    public static final double ksVolts = 0.12356;
    public static final double kvVoltSecondsPerMeter = 1.35;
    public static final double kaVoltSecondsSquaredPerMeter = 0.34;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 1.827;
    // public static final double kDDriveVel = 4.7314;
    public static final int neoCountsPerRevolution = 42;
    public static final int FR_ID = 12;
    public static final int BR_ID = 22;
    public static final int FL_ID = 11;

    public static final int BL_ID = 21;
    public static final double rpm_to_ms_wheel_converter = (Math.PI / 30) * Units.inchesToMeters(3);
    public static final double converter_distance_from_rotations = (DriveConstants.neoMotorWheelCircumference
        * DriveConstants.motorGearBoxRatio) / DriveConstants.neoEncoderCountsPerRev;
    public static final double neoEncoderCountsPerRev = 42;
    public static final double neoMotorWheelCircumference = (Math.PI * Units.inchesToMeters(6));
    public static final double motorGearBoxRatio = 7.37;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.01;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
