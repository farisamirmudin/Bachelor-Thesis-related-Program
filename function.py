import numpy as np
from sympy import *
from sympy.geometry import *
import matplotlib.pyplot as plt

def solve(point, distance):
    circles={}
    min_r = min(distance)
    for i in range (0, len(point)):
        circles[f'c{i}'] = Circle(point[i], distance[i])
        if distance[i] == min_r:
            str = f'c{i}'
            smallest_circle = circles[str]
    i_dict = {}
    i = 0
    for key1 in circles:
        for key2 in circles:
            if key1 != key2 and (f"{key1}_{key2}" and f"{key2}_{key1}") not in i_dict.keys():
                poc = intersection(circles[key1], circles[key2])
                if poc:
                    i_dict[f"{key1}_{key2}"] = poc
    length = len(i_dict)
    if length == 1:
        # print("Closest Point Algorithm")
        segments = {}
        for key in i_dict:
            if str not in key:
                for p in i_dict[key]:
                    dist = smallest_circle.center.distance(p)
                    segments[dist] = Segment(smallest_circle.center, p)
        min_dist = min(list(segments))
        pos =  intersection(segments[min_dist], smallest_circle)[0]

    if length == 2:
        # print("Small Circle Intersection")
        poc = list(i_dict.values())
        cross_i = {}
        for p1 in poc[0]:
            for p2 in poc[1]:
                cross_i[p1.distance(p2)] = [p1, p2]
        min_dist = min(list(cross_i))
        for point in cross_i[min_dist]:
            if not intersection(point,smallest_circle):
                segment = Segment(smallest_circle.center, point)
                pos = intersection(segment, smallest_circle)[0]
                break

    if length == 3:
        segment=[]
        j = 0
        for key in i_dict:
            poc = i_dict[key]
            segment.append(Segment(poc[j], poc[j+1]))
        concurrent = Segment.are_concurrent(segment[0], segment[1], segment[2])
        if concurrent:
            # print("Line Intersection Algorithm")
            pos = intersection(segment[0], segment[1])[0]
        else:
            # print("Comparison Approach of Intersection Distances")
            poc = []
            for key in i_dict:
                if str in key:
                    poc.extend(i_dict[key])
            d1 = poc[0].distance(poc[2])
            d2 = poc[1].distance(poc[3])
            if d1 < d2:
                pos = poc[0].midpoint(poc[2])
            else:
                pos = poc[1].midpoint(poc[3])

    return np.array([pos.x, pos.y])
'''
def trilateration(points, distance, opt='modified'):
    points = np.array(points)
    distance = np.array(distance)
    d1, d2, d3, d4 = distance
    p1, p2, p3, p4 = points

    a1, b1, c1 = p1
    a2, b2, c2 = p2
    a3, b3, c3 = p3
    a4, b4, c4 = p4

    if opt == 'modified':
        e1 = np.linalg.norm(p1)**2
        e2 = np.linalg.norm(p2)**2
        e3 = np.linalg.norm(p3)**2
        C = np.array([[d1**2-d2**2-e1+e2],
                    [d1**2-d3**2-e1+e3]])
        A = np.array([[2*(a2-a1), 2*(b2-b1)],
                    [2*(a3-a1), 2*(b3-b1)]])
        x = np.linalg.lstsq(A, C, rcond=None)[0]
        x = np.reshape(x, 2)
        p4_xy = np.array([a4, b4])
        hori_dist = np.linalg.norm(x-p4_xy)
        x = np.array([x[0], x[1], 0])
        if d4 > hori_dist:
            vert_dist = np.sqrt(d4**2 - hori_dist**2)
            x[2] = vert_dist
    elif opt == 'matrix':
        e1 = np.linalg.norm(p1)**2
        e2 = np.linalg.norm(p2)**2
        e3 = np.linalg.norm(p3)**2
        e4 = np.linalg.norm(p4)**2
        C = np.array([[d1**2-d2**2-e1+e2],
                    [d1**2-d3**2-e1+e3],
                    [d1**2-d4**2-e1+e4]])
        A = np.array([[2*(a2-a1), 2*(b2-b1), 2*(c2-c1)],
                    [2*(a3-a1), 2*(b3-b1), 2*(c3-c1)],
                    [2*(a4-a1), 2*(b4-b1), 2*(c4-c1)]])
        x = np.linalg.lstsq(A, C, rcond=None)[0]
        x = np.reshape(x, 3)
        if x[2] < 0:
            x[2] = 0
        # x[2] = abs(x[2])
    return x
'''
def trilaterate3D(points, distances):
    p1,p2,p3,p4 = np.array(points)
    r1,r2,r3,r4 = np.array(distances)
    e_x=(p2-p1)/np.linalg.norm(p2-p1)
    i=np.dot(e_x,(p3-p1))
    e_y=(p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
    e_z=np.cross(e_x,e_y)
    d=np.linalg.norm(p2-p1)
    j=np.dot(e_y,(p3-p1))
    x=((r1**2)-(r2**2)+(d**2))/(2*d)
    y=(((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
    z1=np.sqrt(max(0, r1**2-x**2-y**2))
    z2=-z1
    ans1=p1+(x*e_x)+(y*e_y)+(z1*e_z)
    ans2=p1+(x*e_x)+(y*e_y)+(z2*e_z)
    dist1=np.linalg.norm(p4-ans1)
    dist2=np.linalg.norm(p4-ans2)
    if np.abs(r4-dist1)<np.abs(r4-dist2):
        if ans1[2] < 0:
            ans1[2] = 0
        return ans1
    else: 
        if ans2[2] < 0:
            ans2[2] = 0
        return ans2
def trilaterate2D(points, distances):
    p1,p2,p3 = np.array(points)
    r1,r2,r3 = np.array(distances)
    e_x=(p2-p1)/np.linalg.norm(p2-p1)
    i=np.dot(e_x,(p3-p1))
    e_y=(p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
    d=np.linalg.norm(p2-p1)
    j=np.dot(e_y,(p3-p1))
    x=((r1**2)-(r2**2)+(d**2))/(2*d)
    y=(((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
    return x, y

def error_check():
    x = np.array([25, 50, 75, 100, 150, 200, 250, 300, 350, 400, 450, 500])
    real_x = np.array([42.81, 72., 102.17, 131.43, 177.12, 234.79, 286.28, 341.28, 385.97, 433.88, 489.53, 544.13])
    e = real_x - x
    m, b = np.polyfit(x, e, 1)
    return m, b

