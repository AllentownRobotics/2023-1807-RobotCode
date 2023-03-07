// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS.LowLevelCMDS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utils.Enums.PlacementType;
import frc.robot.subsystems.Arm;

public class SetCubeOrCone extends CommandBase {
  Arm arm;
  CommandXboxController controller;

  public SetCubeOrCone(Arm arm, CommandXboxController controller) {
    this.arm = arm;
    this.controller = controller;
  }

  @Override
  public void execute(){
    PlacementType type;
    if (controller.leftBumper().getAsBoolean()){
      type = PlacementType.Cube;
    }
    else{
      type = PlacementType.Cone;
    }
    arm.setPlaceType(type);
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}
