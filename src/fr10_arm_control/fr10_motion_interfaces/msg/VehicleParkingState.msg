uint8 FREE=0
uint8 HOLD_REQUESTED=1
uint8 STATIONARY=2
uint8 EXPIRED_LATCHED=3

std_msgs/Header header
uint64 owner_id
uint64 sequence
uint8 state
uint64 rejected_drive_commands
