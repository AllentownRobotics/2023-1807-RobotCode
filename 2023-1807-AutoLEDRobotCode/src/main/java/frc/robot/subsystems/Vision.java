// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  double x;
  double y;
  double area;
  double[] dis;
  double distance;
  double distancex;

  @Override
  public void periodic() {
      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      area = ta.getDouble(0.0);
      dis = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
      distance = dis[1];
      distancex = dis[0];
      SmartDashboard.putNumberArray("Distance array", dis);
      SmartDashboard.putNumber("distance", distance);
      SmartDashboard.putNumber("LimelightX", x);
      SmartDashboard.putNumber("LimelightY", y);
  }

  public double getVisionX()
  {
      return x;     
  }

  public double getVisionY()
  {
      return y;     
  }

  public double getDistance()
  {
    return distance;
  }

  public double getDistanceX()
  {
    return distancex;
  }
}
