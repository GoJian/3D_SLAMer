# /mapping/health  (optional Gaussian mapper, M9)
uint8 STATE_DISABLED = 0
uint8 STATE_RUNNING  = 1
uint8 STATE_PAUSED   = 2            # paused for latency, thermal or memory pressure
uint8 STATE_FAULT    = 3

std_msgs/Header header
uint8 state
float32 queue_age_s
uint32 queue_length
uint32 active_submaps
uint64 gaussian_count
float32 optimization_time_ms
uint64 memory_bytes
