float32[] t         # fine time
float32[] t_coarse  # coarse time
# float32[] t_original # original time

float32[] omega0    # computed body rotation speed input commands
float32[] v0        # computed wheel velocity input commands

float32[] omega0_coarse    # computed body rotation speed input commands
float32[] v0_coarse        # computed wheel velocity input commands

float32[] px1        # computed trailer x-position trajectory, sub-sampled
float32[] py1        # computed trailer y-position trajectory, sub-sampled
float32[] theta1     # computed trailer body orientation states, sub-sampled
float32[] theta0     # computed amr body orientation states, sub-sampled

float32[] px1_coarse        # computed trailer x-position trajectory
float32[] py1_coarse        # computed trailer y-position trajectory
float32[] theta1_coarse     # computed trailer body orientation states
float32[] theta0_coarse     # computed amr body orientation states


uint16[]  stage_indices  # indices at which stage transitions - first element is begin second stage.
bool      success        # computation successful
