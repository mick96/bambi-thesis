########################### FILE: Field.msg ###########################
bambi_msgs/GeoPosition2D[] boundary_path

####################### FILE: GeoPosition2D.msg #######################
float64 latitude
float64 longitude

######################### FILE: NavSatFix.msg #########################
# Navigation Satellite fix for any Global Navigation Satellite System
# Specified using the WGS 84 reference ellipsoid
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type

######################### FILE: OrthoPhoto.msg #########################
string filenameWithFullPath
