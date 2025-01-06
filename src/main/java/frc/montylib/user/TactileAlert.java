// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.montylib.user;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**Creates a tactile alert using a controller's ability to rumble */
public class TactileAlert extends Command {
    private final CommandXboxController controller;

    /**
     * Creates a new instace of TactileAlert, where the .withTimout() function controls the duration of the controller's rumble
     * @param controller the CommandXboxController instance in which a rumble command will be sent to
     */
    public TactileAlert(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public void execute() {
        controller.getHID().setRumble(RumbleType.kBothRumble, 1.0); //Sets rumble at full power
    }

    @Override
    public void end(boolean interrupted) {
        controller.getHID().setRumble(RumbleType.kBothRumble, 0.0); //Stops rumble
    }
}
