import numpy as np
import heapq
import matplotlib.pyplot as plt
from geopy import distance
from matplotlib.patches import Rectangle
import matplotlib.transforms as transforms
from geopy.distance import geodesic

def getGPSDistance(GPS1, GPS2):
    return distance.geodesic(GPS1, GPS2).meters

def getGPSfromENU(refGPS, pointENU):
    dx, dy = pointENU

    # Compute new GPS coordinate using geodesic
    new_point = geodesic(meters=dy).destination(geodesic(meters=dx).destination(refGPS, 90), 0)
    new_lat = new_point.latitude
    new_lon = new_point.longitude

    return (new_lat, new_lon)    


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

def distances(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

if __name__ == "__main__":
    droneGPS = (35.8882176, 128.6064571)
    pointGPS = (35.8882979, 128.6065479)
    delta_east = getGPSDistance((droneGPS[0], pointGPS[1]), droneGPS)
    delta_north = getGPSDistance((droneGPS[0], pointGPS[1]), pointGPS)
    dist = getGPSDistance(droneGPS, pointGPS)
    print(getGPSDistance((0,0), pointGPS))
    print(getGPSDistance((0,0), pointGPS))
    print(np.sqrt(delta_east**2+ delta_north**2))
    print(delta_east, delta_north)
    print(dist)
    print(np.sqrt(10**2+ 20**2))
    print(np.degrees(np.arctan2(1,np.sqrt(3))))