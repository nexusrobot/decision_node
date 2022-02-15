# -*- coding: utf-8 -*-
"""
Created on Tue Feb 15 13:38:25 2022

@author: 4080076
"""
import math as m

def loadWaypoints():
    with open("waypoints.txt","r") as fp:
        lines = fp.read().splitlines()
        waypoints = [tuple(map(float,line.split(','))) for line in lines]
        return waypoints

#現在地から一番近いwaypointを探し、そのpointを返す
def nearest(x0, y0, waypoints):
    cost = []
    for w in waypoints:
        dist = m.sqrt((w[0] - x0)**2 + (w[1] - y0)**2)
        cost.append((dist, w))
    # print(cost)
    #point = sorted(cost, key = lambda x: x[0])[0][1]
    point = min(cost, key = lambda x: x[0])[1]
    
    return point


# current position
x = 1.2
y = 0.8

w = loadWaypoints()
n = nearest(x,y,w)

print(n)


