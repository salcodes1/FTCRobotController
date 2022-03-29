package org.firstinspires.ftc.teamcode.bt;

import java.util.HashMap;

public abstract class Action implements Cloneable {

    /**
     * The array of child actions an action may contain
     */
    protected Action[] childActions;


    public enum State {
        DEFAULT,
        STARTED,
        TICKED,
        ENDED
    }


    HashMap<State, Long> lastStateChange = new HashMap<>();


    State state = State.DEFAULT;

    // start runs once, starts the action

    /**
     * The outer method of starting the action. Should be ran exactly once, and never ran again
     * until {@link #end(AutonomousOpMode)}
     * @param context the autonomous opmode the action is running in
     */

    public void start(AutonomousOpMode context) {
        assert(state == State.DEFAULT || state == State.ENDED);
        state = State.STARTED;
        lastStateChange.put(State.STARTED, System.currentTimeMillis());
        _start(context);
    }

    /**
     * Actual implementation of {@link #start(AutonomousOpMode)}
     */

    protected abstract void _start(AutonomousOpMode context);

    /**
     * Should be called as frequently as possible during the action's running state.
     * @param context the autonomous opmode the action is running in
     */

    public void tick(AutonomousOpMode context) {
        assert(state == State.STARTED || state == State.TICKED);
        state = State.TICKED;
        lastStateChange.put(State.TICKED, System.currentTimeMillis());
        _tick(context);
    }

    /**
     * Actual implementation of {@link #tick(AutonomousOpMode)}
     */

    protected abstract void _tick(AutonomousOpMode context);

    /**
     * @param context the autonomous opmode the action is running in
     * @return whether the action considers it has finished its task
     */

    public boolean hasFinished(AutonomousOpMode context) {
        return _hasFinished(context);
    }

    /**
     * Actual implementation of {@link #hasFinished(AutonomousOpMode)}
     */

    protected abstract boolean _hasFinished(AutonomousOpMode context);

    /**
     * Called to clean up the action
     *  (for example stop any motors that have been ran)
     * @param context the autonomous opmode the action is running in
     */

    public void end(AutonomousOpMode context) {
        assert(state == State.STARTED || state == State.TICKED);
        state = State.ENDED;
        lastStateChange.put(State.ENDED, System.currentTimeMillis());
        _end(context);
    }

    /**
     * Actual implementation of {@link #end(AutonomousOpMode)}
     */

    protected abstract void _end(AutonomousOpMode context);

    /**
     * @return Returns an array of all actions the actions is managing the lifecycle of
     */
    public Action[] getChildActions() {
        return childActions;
    }

    /**
     * @return The action's current state
     */

    public State getState() {
        return state;
    }

    /**
     * @param state The state querying for
     * @return The last time the queried state has been achieved
     */
    public long getLastTimeStampForState(State state) {
        return lastStateChange.getOrDefault(state, (long) -1);
    }

    public boolean DEBUG_showChildren() { return true; }
    
    public Object clone() throws CloneNotSupportedException {
        return super.clone();
    }

    public String getCustomDisplay() {
        return "";
    }

}
