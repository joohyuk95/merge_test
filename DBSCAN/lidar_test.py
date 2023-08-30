#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
from sklearn.cluster import DBSCAN
import numpy as np

def scan_callback(scan):
    ranges = scan.ranges
    obstacle_points = []
    threshold = 15  # 장애물 탐지 거리 [m]

    for i, distance in enumerate(ranges):
        # if distance < scan.range_max:
        if distance < threshold:
            # 장애물의 좌표 계산
            angle = scan.angle_min + i * scan.angle_increment
            x = distance * np.sin(angle)
            y = distance * np.cos(angle)
            obstacle_points.append([x, y])

    if obstacle_points:
        obstacle_points = np.array(obstacle_points)
        # dbscan = DBSCAN(eps=0.5, min_samples=3)  # DBSCAN 파라미터 수정 필요할 때 여기
        dbscan = DBSCAN(eps=3.0, min_samples=5)  # DBSCAN 파라미터 수정 필요할 때 여기
        labels = dbscan.fit_predict(obstacle_points)
        # labels = dbscan.fit(obstacle_points)
        print('labels : ')
        print(labels)

        num_clusters = len(np.unique(labels))
        print("감지된 장애물 개수:", num_clusters)
        # for num in range(num_clusters):
            # print(f"{num+1}번째 cluster의 points:")
            # print(obstacle_points[labels==num])
        
        # 감지된 장애물 개수 2개일 때만 출력
        if num_clusters == 2:
            unique_labels = np.unique(labels)

            cluster_points1 = obstacle_points[labels == 0]
            cluster_center1 = np.mean(cluster_points1, axis=0)
            cluster_points2 = obstacle_points[labels == 1]
            cluster_center2 = np.mean(cluster_points2, axis=0)

            if cluster_center1[0] > cluster_center2[0]:
                points_temp = cluster_points1
                cluster_points1 = cluster_points2
                cluster_points2 = points_temp

                center_temp = cluster_center1
                cluster_center1 = cluster_center2
                cluster_center2 = center_temp

            print("클러스터 중심 좌표1:", cluster_center1)
            print("클러스터 중심 좌표2:", cluster_center2)
            print('cluster1 coordinates')
            print(cluster_points1)
            print('cluster2 coordinates')
            print(cluster_points2)
            
            # for obstacle_point in obstacle_points:
            #     x = obstacle_point[0]
            #     y = obstacle_point[1]
                # print("x : ", x, "y : ", y)

rospy.init_node('lidar_subscriber')
rospy.Subscriber('/laser/scan', LaserScan, scan_callback)
rospy.spin()
