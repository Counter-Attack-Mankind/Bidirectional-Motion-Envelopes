std_msgs/Header header

int8 PREPARING=0
int8 VALIDATED=1
int8 STARTED=2
int8 SEGMENT_CHANGED=3
int8 REACHED=4
int8 FAILED=5

uint32 task_id
int32 vehicle_id
int8 status
string reason
string result_directory
