package frc.alotobots.library.subsystems.swervedrive;

public enum ModulePosition {
    FRONT_LEFT(0),
    FRONT_RIGHT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3);

    public final int index;

    ModulePosition(int index) {
        this.index = index;
    }
}
