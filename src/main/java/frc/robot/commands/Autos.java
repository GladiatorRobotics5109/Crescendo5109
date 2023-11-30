// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Common;
import frc.robot.Constants;

public class Autos {
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  private static SendableChooser<String> m_autoChooser = new SendableChooser<String>();

  private static Command m_defaultAutoCommand;

  public static void init() {
    // PUT THIS ON CONSTRUCTOR OF EACH COMMAND LATER
    // load trajectory from pathweaver json
    String trajectoryJSONPath = "paths/YourPath.wpilib.json";

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONPath);
      Common.currentAutonTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } 
    catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONPath, ex.getStackTrace());
    }



    m_autoChooser.setDefaultOption("Default Auto", Constants.Autos.kDefaultAuto);
    // m_autoChooser.addOption() as necessary

    SmartDashboard.putData("Auto Choice", m_autoChooser);
  }

  public static Command getCurrentAutoCommand() {
    switch (m_autoChooser.getSelected()) {
      case Constants.Autos.kDefaultAuto:
        return m_defaultAutoCommand;
      default:
        return m_defaultAutoCommand;
    }
  }
}
