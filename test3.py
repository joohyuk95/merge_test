import math
from geomag import WMM

def gps_bearing(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    
    # Calculate differences in latitude and longitude
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    # Calculate the bearing
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.atan2(y, x)

    # Convert the bearing from radians to degrees and normalize to [0, 360)
    angle = math.degrees(bearing)
    return (angle + 360) % 360

def magnetic_declination(lat, lon):
    # Set up the World Magnetic Model
    wmm = WMM()

    # Calculate the magnetic declination at the given latitude and longitude
    mag_info = wmm.calc_mag_field(lat, lon, 0, alt=0)
    declination = mag_info.decl
    return declination

def corrected_angle(true_angle, declination):
    # Normalize the magnetic angle to [0, 360)
    magnetic_angle = (true_angle + declination + 360) % 360
    return magnetic_angle

# Example coordinates for points A and B
latA, lonA = 40.71, -74.00
latB, lonB = 40.72, -74.01

# Calculate the true angle between points A and B and the True North direction
true_angle = gps_bearing(latA, lonA, latB, lonB)

# Calculate the magnetic declination at point A
declination = magnetic_declination(latA, lonA)

# Correct the true angle to get the angle between points A and B and the Magnetic North direction
magnetic_angle = corrected_angle(true_angle, declination)

print("Magnetic angle (degrees):", magnetic_angle)


latA, lonA = 47.3977419, 8.5455939  # Example coordinates for point A
latB, lonB = 47.3979032, 8.5460695  # Example coordinates for point B


angle = gps_bearing(latA, lonA, latB, lonB)
print("Angle (degrees):", angle)
