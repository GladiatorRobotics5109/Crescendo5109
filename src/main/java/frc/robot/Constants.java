// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kTurnMotorGearRatio = 12.8;

  public static final double kJoystickDeadzone = 0.1;

  public static class DriveTeamConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    private final double m_maxAngularSpeed;
    private final double m_maxSpeed; // m/s

    private final AHRS m_navX;

    public SwerveConstants(
        double maxAngularSpeed, 
        double maxSpeed,
        SwerveModule frontLeft, 
        SwerveModule frontRight, 
        SwerveModule backLeft, 
        SwerveModule backRight,
        AHRS navX) {
            m_maxAngularSpeed = maxAngularSpeed;
            m_maxSpeed = maxSpeed;

            m_frontLeft = frontLeft;
            m_frontRight = frontRight;
            m_backLeft = backLeft;
            m_backRight = backRight;

            m_navX = navX;
    }

    public SwerveModule getFrontLeft() {
        return m_frontLeft;
    }
    public SwerveModule getFrontRight() {
        return m_frontRight;
    }
    public SwerveModule getBackLeft() {
        return m_backLeft;
    }
    public SwerveModule getBackRight() {
        return m_backRight;
    }

    public AHRS getNavX() {
        return m_navX;
    }

    public double getMaxSpeed() {
      return m_maxSpeed;
    }
    public double getMaxAngluarSpeed() {
      return m_maxAngularSpeed;
    }
  }
}