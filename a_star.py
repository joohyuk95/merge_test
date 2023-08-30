import numpy as np
import heapq
import matplotlib.pyplot as plt
from geopy import distance
from matplotlib.patches import Rectangle
import matplotlib.transforms as transforms
from geopy.distance import geodesic

obstacle_cell_set = set()
obstacle_margin_cell_set = set()

def getENUfromGPS(sourceGPS, targetGPS):
    middleGPS = (sourceGPS[0], targetGPS[1])

    delta_east = getGPSDistance(middleGPS, sourceGPS)
    delta_north = getGPSDistance(middleGPS, targetGPS)

    if sourceGPS[0] > targetGPS[0]:
        delta_north *= -1
    if sourceGPS[1] > targetGPS[1]:
        delta_east *= -1

    ENU = (delta_east, delta_north)

    return ENU

def getGPSDistance(GPS1, GPS2):
    return distance.geodesic(GPS1, GPS2).meters

def getGPSfromENU(refGPS, pointENU):
    dx, dy = pointENU

    # Compute new GPS coordinate using geodesic
    new_point = geodesic(meters=dy).destination(geodesic(meters=dx).destination(refGPS, 90), 0)
    new_lat = round(new_point.latitude, 7)
    new_lon = round(new_point.longitude, 7)

    return (new_lat, new_lon)

def getGPSpath(path, originGPS, globalRotation):
    GPSpath = []
    for node in path:
        ENUpoint = transformation(globalRotation, (0, 0), node)
        GPSpoint = getGPSfromENU(originGPS, ENUpoint)
        GPSpath.append(GPSpoint)

    return GPSpath

def polar2cartesian(lidarPoint):
    r, theta = lidarPoint    # theta range [0, 2pi]
    theta = np.radians(theta)

    x = r*np.sin(theta)
    y = r*np.cos(theta)
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
    print(np.degrees(pointCompass))
    print(droneHeading)
    print(np.degrees(theta))
    x = dist*np.sin(theta)
    y = dist*np.cos(theta)
    cartesian = (x, y)

    return cartesian

def getTransform(wpt, obstacle1, obstacle2):
    if obstacle1[0] > obstacle2[0]:
        obs1 = obstacle2
        obs2 = obstacle1
    else:
        obs1 = obstacle1
        obs2 = obstacle2

    A = np.array([obs2[0]-obs1[0], obs2[1]-obs1[1]])
    B = np.array([-wpt[0], -wpt[1]])
    C = np.array([1, 0])

    if np.cross(A, B) > 0:
        angle = np.arctan2(np.linalg.norm(np.cross(A, B)), np.dot(A, B))    # drone과 wpt#2를 이은 직선과 장애물 직선 각도
    else:
        angle = (-1)*np.arctan2(np.linalg.norm(np.cross(A, B)), np.dot(A, B))
    dist = np.linalg.norm(wpt)

    translation = (dist*np.cos(angle), dist*np.sin(angle))

    if np.cross(A, C) > 0:
        obs2bodyAngle = np.arctan2(np.linalg.norm(np.cross(A, C)), np.dot(A, C))    # 장애물 직선과 드론 x-axis
    else:
        obs2bodyAngle = (-1)*np.arctan2(np.linalg.norm(np.cross(A, C)), np.dot(A, C))

    transform = [translation, obs2bodyAngle]

    return transform

def transformation(rotaionAngle, translation, point):
    rotation_matrix = np.array([[np.cos(rotaionAngle), -np.sin(rotaionAngle)],
                                [np.sin(rotaionAngle), np.cos(rotaionAngle)]])
    
    transformed = np.dot(rotation_matrix, point) + translation

    return tuple(transformed)


def create_grid_map(grid_range, obstacles):
    grid_x_size = grid_range[0][1] - grid_range[0][0] + 1
    grid_y_size = grid_range[1][1] - grid_range[1][0] + 1

    max_neighbors = 8   # this is 8 when the diagonal movement is possible, or 4(left, right, up, down); the order starts from right and couter-clockwise direction
    infinity = float('inf')
    cell_size = 1   # [m]
    map = np.full((grid_x_size, grid_y_size, max_neighbors), cell_size, dtype=float)

    diagonal_distance = np.sqrt(cell_size)
    map[:,:,1] = map[:,:,3] = map[:,:,5] = map[:,:,7] = diagonal_distance
    
    neighbors = [(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]
    obstacle_set = set()
    obstacle_margin_set = set()
    for obstacle in obstacles:
        x_obs, y_obs = obstacle
        obstacle_set.add((int(np.floor(x_obs)), int(np.floor(y_obs))))
        obstacle_set.add((int(np.ceil(x_obs)), int(np.floor(y_obs))))
        obstacle_set.add((int(np.ceil(x_obs)), int(np.ceil(y_obs))))
        obstacle_set.add((int(np.floor(x_obs)), int(np.ceil(y_obs))))
        
        obstacle_margin_set.add((int(np.floor(x_obs)), int(np.floor(y_obs))))
        obstacle_margin_set.add((int(np.ceil(x_obs)), int(np.floor(y_obs))))
        obstacle_margin_set.add((int(np.ceil(x_obs)), int(np.ceil(y_obs))))
        obstacle_margin_set.add((int(np.floor(x_obs)), int(np.ceil(y_obs))))

        obstacle_cell_set.add((int(np.floor(x_obs)), int(np.floor(y_obs))))

    for point in obstacle_set:
        for dx, dy in neighbors:
            neighbor = point[0] + dx, point[1] + dy
            if grid_range[0][0] <= neighbor[0] < grid_range[0][1]:
                if grid_range[1][0] <= neighbor[1] < grid_range[1][1]:
                    obstacle_margin_set.add(neighbor)
                    if point in obstacle_cell_set and neighbor not in obstacle_cell_set:
                        obstacle_margin_cell_set.add(neighbor)
                else:
                    continue
            else:
                continue            
    
    for point in obstacle_margin_set:
        for i, (dx, dy) in enumerate(neighbors):
            neighbor = point[0] + dx, point[1] + dy
            if grid_range[0][0] <= neighbor[0] <= grid_range[0][1]:
                if grid_range[1][0] <= neighbor[1] <= grid_range[1][1]:
                    map[point[0], point[1], i] = infinity
                    if i <= 3:
                        j = i + 4
                    else:
                        j = i - 4
                    map[neighbor[0], neighbor[1], j] = infinity # bidirectional
                else:
                    continue
            else:
                continue  

    return map

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(map, grid_range, start, goal):
    neighbors = [(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]
    close_set = set()
    came_from = {}  # save the shortest path sequence using dictionary {node A :  node B} (A is come from B) = path sequence B -> A
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}    # fscore = gscore + heuristic
    pq = [(fscore[start], start)]   # priority queue [score, (x, y)],  *dictionary indexing 'dictionary[key]'

    while pq:
        current = heapq.heappop(pq)[1] # current node (x, y)
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(current)
            return data[::-1]   # reverse the sequence expression ::-1
        close_set.add(current)  # the smallest F score node (the highest priority)
        for i, (dx, dy) in enumerate(neighbors):
            neighbor = current[0] + dx, current[1] + dy # tuple type
            tentative_g_score = gscore[current] + map[current[0], current[1], i]  # g_score for neighbor
            if grid_range[0][0] <= neighbor[0] <= grid_range[0][1]:     # boundary check
                if grid_range[1][0] <= neighbor[1] <= grid_range[1][1]:
                    pass
                else:
                    continue
            else:
                continue

            # map bound 내에 있는 것만 통과
            if neighbor in close_set:
                continue

            if gscore.get(neighbor) is None:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(pq, (fscore[neighbor], neighbor))

            if tentative_g_score >= gscore.get(neighbor):
                continue

            else:
                pq.remove((fscore[neighbor],neighbor))
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(pq, (fscore[neighbor], neighbor))

    return None

def visualize(grid_range, path, start, goal, obstacles, drone):
    x_min, x_max = grid_range[0][0], grid_range[0][1]
    y_min, y_max = grid_range[1][0], grid_range[1][1]

    plt.figure(figsize=(6, 6))
    # Create points and plot them with connecting lines
    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            plt.scatter(x, y, color="black", zorder=2)  # Plot points

            if x < x_max:
                plt.plot([x, x + 1], [y, y], color="black", zorder=1)  # Plot horizontal lines

            if y < y_max:
                plt.plot([x, x], [y, y + 1], color="black", zorder=1)  # Plot vertical lines

            if x < x_max and y < y_max:
                plt.plot([x, x + 1], [y, y + 1], color="black", zorder=1)  # Diagonal up-right
            
            if x < x_max and y > y_min:
                plt.plot([x, x + 1], [y, y - 1], color="black", zorder=1)  # Diagonal down-right
            
    for obstacle_cell in obstacle_cell_set:     # visualize the cells occupied by obstacles
        plt.gca().add_patch(plt.Rectangle(obstacle_cell, 1, 1, fill=True, color=(0.2,0.2,0.2), zorder=0))
    for obstacle_margin_cell in obstacle_margin_cell_set:
        plt.gca().add_patch(plt.Rectangle(obstacle_margin_cell, 1, 1, fill=True, color=(0.5,0.5,0.5), zorder=0))
    
    x_obs = [obstacle[0] for obstacle in obstacles]
    y_obs = [obstacle[1] for obstacle in obstacles]
    plt.scatter(x_obs, y_obs, s=10, color="orange", zorder=2, label="clustering center of obstacles")

    plt.scatter(*start, color=(1,0,0), zorder=2, label="Start Node")
    plt.scatter(*goal, color=(0,1,0), zorder=2, label="Goal Node")
    plt.scatter(*drone, color=(0,0,1), zorder=2, label="Drone")

    prev_node = None
    for node in path:
        if prev_node is None:
            prev_node = node
            continue

        if prev_node == start:    
            plt.plot([prev_node[0], node[0]], [prev_node[1], node[1]], color="magenta", zorder=2, label="Shortest path")
        else:
            plt.plot([prev_node[0], node[0]], [prev_node[1], node[1]], color="magenta", zorder=2)
        prev_node = node

    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Grid Map in obstacle line coordinate system")

    plt.axis([x_min - 0.5, x_max + 0.5, y_min - 0.5, y_max + 0.5])
    plt.axis('equal') 

    plt.legend()
    # plt.show()
    
def transformedVisualize(grid_range, path, start, goal, obstacles, drone, globalRotation):
    x_min, x_max = grid_range[0][0], grid_range[0][1]
    y_min, y_max = grid_range[1][0], grid_range[1][1]

    plt.figure(figsize=(6, 6))
    # Create points and plot them with connecting lines
    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            plt.scatter(*transformation(globalRotation, (0,0), (x, y)), color="black", zorder=2)  # Plot points

            if x < x_max:
                point1 = transformation(globalRotation, (0,0), (x, y))
                point2 = transformation(globalRotation, (0,0), (x + 1, y))
                plt.plot([point1[0], point2[0]] ,[point1[1], point2[1]], color="black", zorder=1)  # Plot horizontal lines

            if y < y_max:
                point1 = transformation(globalRotation, (0,0), (x, y))
                point2 = transformation(globalRotation, (0,0), (x, y + 1))
                plt.plot([point1[0], point2[0]] ,[point1[1], point2[1]], color="black", zorder=1) # Plot vertical lines

            if x < x_max and y < y_max:
                point1 = transformation(globalRotation, (0,0), (x, y))
                point2 = transformation(globalRotation, (0,0), (x + 1, y + 1))
                plt.plot([point1[0], point2[0]] ,[point1[1], point2[1]], color="black", zorder=1) # Diagonal up-right
            
            if x < x_max and y > y_min:
                point1 = transformation(globalRotation, (0,0), (x, y))
                point2 = transformation(globalRotation, (0,0), (x + 1, y - 1))
                plt.plot([point1[0], point2[0]] ,[point1[1], point2[1]], color="black", zorder=1) # Diagonal down-right                
            
    ax = plt.gca()
    for obstacle_cell in obstacle_cell_set:     # visualize the cells occupied by obstacles
        patch = Rectangle(obstacle_cell, 1, 1, color=(0.2,0.2,0.2))
        rotation_transform = transforms.Affine2D().rotate(globalRotation)
        patch.set_transform(rotation_transform + ax.transData)
        ax.add_patch(patch)
    for obstacle_margin_cell in obstacle_margin_cell_set:
        patch = Rectangle(obstacle_margin_cell, 1, 1, color=(0.5,0.5,0.5))
        rotation_transform = transforms.Affine2D().rotate(globalRotation)
        patch.set_transform(rotation_transform + ax.transData)
        ax.add_patch(patch)
    
    x_obs = [transformation(globalRotation, (0,0), obstacle)[0] for obstacle in obstacles]
    y_obs = [transformation(globalRotation, (0,0), obstacle)[1] for obstacle in obstacles]
    plt.scatter(x_obs, y_obs, s=10, color="orange", zorder=2, label="clustering center of obstacles")
    # plt.scatter(*transformation(globalRotation, (0,0), start), color=(1,0,0), zorder=2, label="Start Node")
    # plt.scatter(*transformation(globalRotation, (0,0), goal), color=(0,1,0), zorder=2, label="Goal Node")
    # plt.scatter(*transformation(globalRotation, (0,0), drone), color=(0,0,1), zorder=2, label="Drone")

    plt.scatter(*transformation(globalRotation, (0,0), (0, 0)), color=(1,0,0), zorder=2, label="Start Node")
    plt.scatter(*transformation(globalRotation, (0,0), (0, y_max)), color=(0,1,0), zorder=2, label="Goal Node")

    # prev_node = None
    # for node in path:
    #     if prev_node is None:
    #         prev_node = node
    #         continue
        
    #     trans_prev_node = transformation(globalRotation, (0,0), (prev_node[0], prev_node[1]))
    #     trans_node = transformation(globalRotation, (0,0), (node[0], node[1]))
        
    #     if prev_node == start:    
    #         plt.plot([trans_prev_node[0], trans_node[0]], [trans_prev_node[1], trans_node[1]], color="magenta", zorder=2, label="Shortest path")
    #     else:
    #         plt.plot([trans_prev_node[0], trans_node[0]], [trans_prev_node[1], trans_node[1]], color="magenta", zorder=2)
    #     prev_node = node

    plt.xlabel("East")
    plt.ylabel("North")
    plt.title("Grid Map in ENU coordinate system")

    plt.axis([x_min - 0.5, x_max + 0.5, y_min - 0.5, y_max + 0.5])
    plt.axis('equal') 

    plt.legend()
    plt.show()

if __name__ == "__main__":
    # grid_range = ((-5, 5), (-7, 7))
    # obstacles = ((-3.7, 0.5), (-3.7, -0.1), (1.2, 3.4), (-1.7, -2.1))
    # map = create_grid_map(grid_range, obstacles)
    # start = (-5,-4)
    # goal = (0,0)
    # shortest_path = astar(map, grid_range, start, goal)
    # visualize(grid_range, shortest_path, start, goal, obstacles)

    # # ex1
    # droneGPS = (35.887760,128.604871)
    # droneHeading = 50
    # wptGPS = (35.887859,128.604906)
    # obs1 = (12, 312.5)
    # obs2 = (12, 346.5)
    # 35.8882979, 128.6065479

    # ex2
    droneGPS = (35.8882176, 128.6064571)
    obs1 = (11.24, 316)
    obs2 = (13.83, 346)
    droneHeading = 75
    wptGPS = (35.8882979, 128.6065479)
    print(getGPSDistance((35.887799, 128.606769), (35.887797,128.606541)))
    print(getGPSDistance((35.887768, 128.606701), (35.887797,128.606541)))
    # ex3
    # droneGPS = (35.8877971, 128.6065414)
    # obs1 = (14.8006, 8)
    # obs2 = (20.5876, 354)
    # droneHeading = 94
    # wptGPS = (35.8877836, 128.6067351)

    # 장애물과 WPT를 body-frame으로 변환하기 위해 x,y로 변환
    body_obs1 = polar2cartesian(obs1)
    body_obs2 = polar2cartesian(obs2)
    body_wpt = LLA2body(wptGPS, droneGPS, droneHeading)
    translation, compass = getTransform(body_wpt, body_obs1, body_obs2)

    rotation = compass
    globalRotation = - (np.radians(droneHeading) + compass)

    gridCenter = transformation(rotation, translation, body_wpt)
    obstacle1 = transformation(rotation, translation, body_obs1)
    obstacle2 = transformation(rotation, translation, body_obs2)
    
    drone = transformation(rotation, translation, (0,0))

    plt.figure(figsize=(6, 6))
    # plt.scatter(*body_wpt, color=(0,1,0), zorder=2, label="WPT#2")
    # plt.scatter(*body_obs1, color="orange", zorder=2, label="obstacles")
    # plt.scatter(*body_obs2, color="orange", zorder=2)
    
    plt.scatter(*(0.011643869068110685, 11.764052486232483), color=(0,1,0), zorder=2, label="WPT#2")
    plt.scatter(*(-2.250099804586113, 9.383076502171486), color="orange", zorder=2, label="obstacles")
    plt.scatter(*(2.818093595315448, 13.841284477959327), color="orange", zorder=2)
    plt.scatter(0,0, color=(0,0,1), zorder=2, label="drone")
    plt.grid()
    plt.title("Grid Map in body-fixed frame")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.legend()

    start1 = (int(np.floor(drone[0])), int(np.ceil(drone[1])))
    start2 = (int(np.ceil(drone[0])), int(np.ceil(drone[1])))
    
    if np.linalg.norm(start1) < np.linalg.norm(start2):
        start = start1
    else:
        start = start2
    goal = (0,0)

    grid_range = ((int(np.floor(obstacle1[0])), int(np.ceil(obstacle2[0]))), (start[1], -start[1]))
    obstacles = (obstacle1, obstacle2)
    map = create_grid_map(grid_range, obstacles)
    shortest_path = astar(map, grid_range, start, goal)
    visualize(grid_range, shortest_path, start, goal, obstacles, drone)
    transformedVisualize(grid_range, shortest_path, start, goal, obstacles, drone, globalRotation)

    GPSpath = getGPSpath(shortest_path, wptGPS, globalRotation)

    delta_east = 3.5*np.cos(np.radians(30))
    delta_north = 3.5*np.sin(np.radians(30))
    print(getGPSfromENU((35.8877836, 128.6067351), (delta_east, delta_north)))
    print(getGPSfromENU((35.8877836, 128.6067351), (-delta_east, -delta_north)))
    
    start = (35.8877971, 128.6065414)
    obs1 = (35.8877678, 128.6067015)
    obs2 = (35.8877994, 128.6067687)
    
    print(getGPSDistance(start, obs1))
    print(getGPSDistance(start, obs2))
    
    print(getGPSfromENU((47.3977419, 8.5455939), (35.865, -17.905)))
    print(getGPSfromENU((47.3977419, 8.5455939), (-35.865, 17.905)))
    print(getGPSfromENU((47.3977419, 8.5455939), (-35.865, -17.905)))

    # print(getGPSDistance((start[0], obs1[1]), start))
    # print(getGPSDistance((start[0], obs1[1]), obs1))  #east

    # print(getGPSDistance((start[0], obs2[1]), start))
    # print(getGPSDistance((start[0], obs2[1]), obs2)) #east
    # print(GPSpath)
    # inputs
    '''
    Drone GPS point = (35.887760,128.604871)
    Drone heading (companss) = 50 deg
    WPT#2 GPS point = (35.887859,128.604906)
    Two obstacles (r,theta) points
    obs1 = (13, 342)
    obs2 = (13, 309)


    # outputs
    body
    transformed wpt#2, obs1, obs2
    global 

    '''