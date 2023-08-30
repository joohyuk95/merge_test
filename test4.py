import numpy as np

def lat_lon_to_radians(lat, lon):
    return np.radians(lat), np.radians(lon)

def bearing_angle(lat1, lon1, lat2, lon2):
    lat1_rad, lon1_rad = lat_lon_to_radians(lat1, lon1)
    lat2_rad, lon2_rad = lat_lon_to_radians(lat2, lon2)
    
    delta_lon = lon2_rad - lon1_rad
    
    y = np.sin(delta_lon) * np.cos(lat2_rad)
    x = np.cos(lat1_rad) * np.sin(lat2_rad) - np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(delta_lon)
    bearing = np.arctan2(y, x)
    
    return np.degrees(bearing)

A = (47.3977419, 8.5455939)
B = (47.3979032, 8.5460695)

bearing = bearing_angle(A[0], A[1], B[0], B[1])
print(bearing)
print(f"Bearing angle between A and B: {bearing:.2f}°")