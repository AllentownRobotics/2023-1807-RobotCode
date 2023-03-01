// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.LowLevelCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Enums.PlacementType;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
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
