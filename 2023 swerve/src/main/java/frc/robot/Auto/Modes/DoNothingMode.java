package frc.robot.Auto.Modes;

import frc.robot.Auto.Extensions.AutoModeEndedException;

import edu.wpi.first.math.geometry.Pose2d;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("doing nothing");
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}
