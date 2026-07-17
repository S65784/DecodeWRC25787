package org.firstinspires.ftc.teamcode.decode.util;

/** Simple gamepad rising-edge detector that also works on SDK versions before 11.1. */
public final class RisingEdge {
    private boolean previous;

    public boolean update(boolean current) {
        boolean pressed = current && !previous;
        previous = current;
        return pressed;
    }
}
