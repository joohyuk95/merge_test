from geographiclib.geodesic import Geodesic
import numpy as np
from geopy import distance
import math
from haversine import haversine
from geopy.distance import geodesic

def getGPSfromENU(refGPS, pointENU):
    lat, lon = refGPS

    # Define the distance (1 meter) and directions (North: 0, East: 90)
    east_distance, north_distance = pointENU
    north_direction = 0  # in degrees
    east_direction = 90  # in degrees

    geod = Geodesic.WGS84
    north_result = geod.Direct(lat, lon, north_direction, north_distance)
    new_lat = north_result['lat2']

    # Calculate the new point after moving east
    east_result = geod.Direct(new_lat, lon, east_direction, east_distance)
    new_lat2 = round(east_result['lat2'], 7)
    new_lon2 = round(east_result['lon2'], 7)

    return (new_lat2, new_lon2)

def getGPSDistance(GPS1, GPS2):
    return distance.geodesic(GPS1, GPS2).meters

def LLA2body(pointGPS, droneGPS, droneHeading):
    delta_east = getGPSDistance((pointGPS[0], droneGPS[1]), pointGPS)
    delta_north = getGPSDistance((droneGPS[0], pointGPS[1]), pointGPS)

    sign_east = pointGPS[1] - droneGPS[1]

    if sign_east > 0:
        pointCompass = np.arctan2(delta_east, delta_north)
    else:
        pointCompass = 2*np.pi + np.arctan2(delta_east, delta_north)

    theta = pointCompass - np.radians(droneHeading)
    if theta > np.pi:
        theta -= 2*np.pi        
    elif theta < np.pi:
        theta += 2*np.pi        
    else:
        pass

    dist = getGPSDistance(pointGPS, droneGPS)
    if theta > 0:
        x, y = dist*np.sin(theta), dist*np.cos(theta)
    else:
        x, y = -dist*np.sin(theta), -dist*np.cos(theta)
    
    cartesian = (x, y)

    return cartesian


def haversine(lat1, lon1, lat2, lon2):
    earth_radius = 6371000  # Earth's radius in meters

    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)

    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = (math.sin(dlat / 2) ** 2) + \
        math.cos(lat1_rad) * math.cos(lat2_rad) * \
        (math.sin(dlon / 2) ** 2)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return earth_radius * c

def east_west_north_south(lat1, lon1, lat2, lon2):

    # Calculate the north-south distance
    dlat = haversine(lat1, lon1, lat2, lon1)
    if lat2 > lat1:
        north_south = dlat
    else:
        north_south = -dlat

    # Calculate the east-west distance
    dlon_temp = haversine(lat1, lon1, lat1, lon2)
    if lon2 > lon1:
        east_west = dlon_temp
    else:
        east_west = -dlon_temp

    return east_west, north_south

def getGPSfromENU2(refGPS, pointENU):
    dx, dy = pointENU

    # Compute new GPS coordinate using geodesic
    new_point = geodesic(meters=dy).destination(geodesic(meters=dx).destination(refGPS, 90), 0)
    new_lat = new_point.latitude
    new_lon = new_point.longitude

    return (new_lat, new_lon)

if __name__ == "__main__":
    gps = (47.3979032, 8.5460695)
    r = 3.5
    theta = np.radians(70)
    enu1 = (r*np.cos(theta), r*np.sin(theta))

    # lat, lon = getGPSfromENU(gps, enu1)
    lat, lon = getGPSfromENU2(gps, enu1)
    print("x distance:", getGPSDistance((lat, gps[1],), (lat, lon)))
    print("y distance:", getGPSDistance((gps[0], lon), (lat, lon)))
    
    print("x distance:", haversine(lat, gps[1], lat, lon))
    print("y distance:", haversine(gps[0], lon, lat, lon))
    # enu2 = (-r*np.cos(theta), -r*np.sin(theta))
    print("this is the x, y on purpose",enu1)

    print(np.degrees(np.arctan2(1,1)))
    print(np.degrees(np.arctan2(1,-1)))
    print(np.degrees(np.arctan2(-1,-1)))
    print(np.degrees(np.arctan2(-1,1)))
    x, y = 1, 2
    print(x, y)


