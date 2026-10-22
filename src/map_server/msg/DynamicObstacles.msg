# Lightweight current perception; independent of EDT completion.
std_msgs/Header header
uint64 producer_id
uint64 input_version
nav_msgs/MapMetaData cloud_info
uint8[] cloud_mask
map_server/RadarTarget[] allies
map_server/RadarTarget[] enemies
map_server/DynamicHealth health
