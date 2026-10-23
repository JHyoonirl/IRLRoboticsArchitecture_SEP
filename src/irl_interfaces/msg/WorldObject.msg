std_msgs/Header header
string object_id
string class_name
geometry_msgs/Pose pose
# /state/*: damped operational reliability, not a calibrated success probability.
float32 confidence
# Detector score is diagnostic; World State confidence is filtered track reliability.
float32 detection_confidence
vision_msgs/BoundingBox2D bbox
bool visible
bool pose_valid
string pose_source
string pose_status
geometry_msgs/Vector3 velocity
float64[9] position_covariance
bool track_confirmed
# Generic observation-quality gate; skill-specific conditions are evaluated elsewhere.
bool actionable
string tracking_status
float32 observation_age_sec
uint32 observation_count
