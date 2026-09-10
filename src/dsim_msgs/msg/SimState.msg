# What the simulator is actually doing, as OBSERVED by dsim_simctl.
#
# Every field here is measured or counted, never an echo of a request. The one
# exception is requested_speed, which is named so it cannot be mistaken for the
# speed the world is achieving -- the two differ whenever the machine cannot
# keep up, and a viewer that showed only the request would be lying at exactly
# the moment the user most needs the truth.
std_msgs/Header header

bool enabled                # false when this build has no control of the world
bool paused
bool pacing                 # true when speed is held by stepping the world

float64 requested_speed     # simulated seconds per wall second, as asked for
float64 achieved_speed      # ...as measured over the last window
float64 min_speed           # the accepted range. Published so the viewer and
float64 max_speed           # the HTTP layer validate against ONE source.

float64 sim_time_s          # simulated time, from the simulator's own stats
float64 step_size_s         # physics step; the quantum of everything above
float64 backlog_s           # simulated seconds the pacer is behind schedule
float64 target_sim_time_s   # the instant the world was last told to run to.
                            # Next to sim_time_s this says whether the world is
                            # honouring the pacing or outrunning it.

uint32 resets               # world resets performed since this node started
uint32 step_errors          # step requests the simulator refused
# Cumulative, so a rate can be differenced out of two samples. Present because
# the pacer is otherwise unobservable: when it ran at twice the requested speed
# the only way to tell was to time the world with a stopwatch.
uint64 ticks                # pacing ticks since launch
uint64 run_requests         # run-to-instant requests sent since launch
string error                # empty when healthy
