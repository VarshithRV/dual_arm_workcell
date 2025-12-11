#include <iostream>
#include <string>

class PoseTrackerStateMachine
{
public:
    enum State : int
    {
        NOT_READY = 0,       // prerequisites not met
        READY = 1,           // prerequisites met, not tracking
        ACTIVE_TRACKING = 2  // actively tracking
    };

    PoseTrackerStateMachine()
        : current_state_(NOT_READY)
    {
    }

    // -------- getter --------
    int getCurrentState() const
    {
        return static_cast<int>(current_state_);
    }

    std::string getCurrentStateName() const
    {
        switch (current_state_)
        {
        case NOT_READY:       return "NOT_READY";
        case READY:           return "READY";
        case ACTIVE_TRACKING: return "ACTIVE_TRACKING";
        default:              return "UNKNOWN";
        }
    }

    // -------- state transitions --------

    // prepare_tracker: NOT_READY -> READY (idempotent)
    void prepareTracker()
    {
        if (current_state_ == NOT_READY)
        {
            current_state_ = READY;
        }
        // if already READY or ACTIVE_TRACKING, do nothing
    }

    // unprepare_tracker: READY or ACTIVE_TRACKING -> NOT_READY
    void unprepareTracker()
    {
        current_state_ = NOT_READY;
    }

    // start_tracker: READY -> ACTIVE_TRACKING
    void startTracker()
    {
        if (current_state_ == READY)
        {
            current_state_ = ACTIVE_TRACKING;
        }
        // if NOT_READY or already ACTIVE_TRACKING, do nothing
    }

    // stop_tracker: ACTIVE_TRACKING -> READY
    void stopTracker()
    {
        if (current_state_ == ACTIVE_TRACKING)
        {
            current_state_ = READY;
        }
        // if NOT_READY or READY, do nothing
    }

private:
    State current_state_;
};

// ---------------- example main ----------------

int main()
{
    PoseTrackerStateMachine sm;

    auto print_state = [&](const std::string &label) {
        std::cout << label
                  << " state = " << sm.getCurrentState()
                  << " (" << sm.getCurrentStateName() << ")\n";
    };

    print_state("Initial");

    sm.prepareTracker();
    print_state("After prepareTracker()");

    sm.startTracker();
    print_state("After startTracker()");

    sm.stopTracker();
    print_state("After stopTracker()");

    sm.unprepareTracker();
    print_state("After unprepareTracker()");

    return 0;
}
