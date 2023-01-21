package frc.robot.Auto;

import frc.robot.Auto.Modes.*;
//import frc.robot.subsystems.ShuffleBoardInteractions; TODO extra credit: Make auto choosing easier than hardcoding it.

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING,
        TEST_PATH
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Test Path", DesiredMode.TEST_PATH);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothingMode());
        case TEST_PATH:
            return Optional.of(new TestPathMode());
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}
