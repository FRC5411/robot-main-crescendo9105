package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;

public class ScrewArmController {
    public final ProfiledPIDController controller;

    public ScrewArmController(ProfiledPIDController controller) {
        this.controller = controller;
    }
}
