Header header
# reserved
uint8 mode
# 0-Unknown 1-P 2-R 3-N 4-D
uint8 gear
# kmh (always positive)
float64 vehicle_speed
# deg (left<0  right>0)
float64 steer_angle
# reserved
float64 yaw_rate
# reserved
float64[] wheel_speed
float64 latitude
float64 longitude
# North-0/360 East-90
float64 azimuth
# Narrow_Int->RTK
string rtk_stat
