#------------------------------Bambi_msg------------------------------#
###########################FieldCoverageInfo###########################

float32 thermal_camera_ground_footprint_width
float32 thermal_camera_ground_footprint_height
# Scanning Altitude
float32 relative_altitude_scanning
# Altitude to use when going back home
float32 relative_altitude_returning
bambi_msgs/GeoPositionWithRelativeAltitude current_position
bambi_msgs/GeoPosition2D home_position
bambi_msgs/Field field

#################################Field#################################

bambi_msgs/GeoPosition2D[] boundary_path

#############################GeoPosition2D#############################

float64 latitude
float64 longitude

####################GeoPositionWithRelativeAltitude####################

bambi_msgs/GeoPosition2D geopos_2d
#Altitude is in meters
float32 altitude_over_ground

#################################Path#################################

bambi_msgs/GeoPositionWithRelativeAltitude[] geometric_path

##############################TerrainData##############################

float64 altitudeInMeters

