package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import static com.pedropathing.ivy.Scheduler.schedule;

/**
 * TelePathing
 *
 * All the "path to a point" logic from TeleOpTest, moved out of the TeleOp.
 *
 * Gamepad 1 controls handled here:
 *   A          -> build a path from where the robot is now to the waypoint and follow it
 *   B          -> abort pathing and give control back to the driver
 *   Any stick  -> also aborts pathing (driver takes over)
 *   Y          -> save the robot's current pose as the new waypoint (only while not pathing)
 *
 * The TeleOp should only run manual driving when isPathing() is false.
 * update() MUST be called every loop.
 */
public class TelePathing {

    private static final double STICK_DEADZONE = 0.1;

    private final Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Target to drive to (field coords, same system as the Autos).
    private Pose waypoint;

    private boolean pathing = false;
    private boolean lastA = false;   // for detecting a fresh A press

    public TelePathing(Follower follower) {
        this.follower = follower;
    }

    /** Call every loop with gamepad1. */
    public void update(Gamepad gp) {

        // ---------- START PATHING (A press) ----------
        // Only on the moment A is pressed (not while held), and only if not already pathing.
        boolean a = gp.a;
        if (a && !lastA && !pathing) {
            Command move = HiveRoutes.goTo(follower, follower.pose(), waypoint);
            if (move != null) {          // null = already at the waypoint
                schedule(move);
                pathing = true;
            }
        }
        lastA = a;

        // ---------- ABORT (B or stick input) ----------
        boolean driverInput = Math.abs(gp.left_stick_x) > STICK_DEADZONE
                || Math.abs(gp.left_stick_y) > STICK_DEADZONE
                || Math.abs(gp.right_stick_x) > STICK_DEADZONE;

        if (pathing && (gp.b || driverInput)) {
            abort();
        }

        // ---------- FINISHED ----------
        if (pathing && !follower.isBusy()) {   // path finished (2.x name; verify)
            pathing = false;
            follower.manual();
        }

        // ---------- SET WAYPOINT (Y) ----------
        if (!pathing && gp.y) {
            Pose p = follower.pose();
            waypoint = poseFactory.of(p.x(), p.y(), Math.toDegrees(p.heading()));
        }
    }

    /** Stop following the path and hand control back to the driver. */
    public void abort() {
        Scheduler.reset();         // clears the scheduled follow command
        pathing = false;
        follower.manual();
    }

    public boolean isPathing()  { return pathing; }
    public Pose getWaypoint()   { return waypoint; }
}