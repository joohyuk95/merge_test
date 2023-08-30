def LLA2body2(pointGPS, droneGPS, droneHeading):
    delta_east = getGPSDistance((droneGPS[0], pointGPS[1]), droneGPS)
    delta_north = getGPSDistance((droneGPS[0], pointGPS[1]), pointGPS)

    sign_east = pointGPS[1] - droneGPS[1]
    sign_north = pointGPS[0] - droneGPS[0]
    
    if sign_east < 0:
        delta_east *= (-1.0)
    if sign_north < 0:
        delta_north *= (-1.0)

    print(delta_east, delta_north)
    if delta_east > 0:
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
    
    x, y = dist*np.sin(theta), dist*np.cos(theta)  
    cartesian = (x, y)

    return cartesian

def LLA2body(pointGPS, droneGPS, droneHeading):
    lat_c, lon_c = pointGPS
    lat_0, lon_0 = droneGPS
    
    delta_lat = lat_c - lat_0
    delta_lon = lon_c - lon_0
    pointCompass = np.arctan2(delta_lon, delta_lat)
    if pointCompass < 0:
        pointCompass += 2*np.pi

    theta = pointCompass - np.radians(droneHeading)
    dist = getGPSDistance(pointGPS, droneGPS)

    x = dist*np.sin(theta)
    y = dist*np.cos(theta)

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