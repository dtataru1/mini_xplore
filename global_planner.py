import numpy as np
import matplotlib.pyplot as plt
import pyvisgraph as vg
from pyvisgraph import visible_vertices as vv
import merge_obstacles as mo
from pyvisgraph.graph import Point
import math
from constants import *

polys = [[vg.Point(31.0,31.0), vg.Point(34.0,31.0), vg.Point(32.0,11.0), vg.Point(29.0,11.0)],
        [vg.Point(10.0,51.0), vg.Point(15.0,85.0), vg.Point(18.0,85.0), vg.Point(13,51.0)],
        [vg.Point(40.0,60.0), vg.Point(60.0,60.0), vg.Point(60.0,63.0), vg.Point(40.0,63.0)]]
        #[vg.Point(62.0,60.0), vg.Point(64.5,63.5), vg.Point(84.0,52.0), vg.Point(84.0,45.0)],
        #[vg.Point(62.0,1.0), vg.Point(65.0,1.0), vg.Point(65.0,15.0), vg.Point(62.0,15.0)],
        #[vg.Point(68.0,28.0), vg.Point(96.0,28.0), vg.Point(96.0,31.0), vg.Point(68,31.0)],
        #[vg.Point(1.0,12.0), vg.Point(1.0,15.0), vg.Point(10.0,15.0), vg.Point(10,12.0)],
        #[vg.Point(10.0,28.0), vg.Point(30.0,28.0), vg.Point(30.0,31.0), vg.Point(10,31.0)],
        #[vg.Point(30.0,1.0), vg.Point(32.0,1.0), vg.Point(32.0,5.0), vg.Point(30,5.0)],
        #[vg.Point(9.0,24.0), vg.Point(9.0,18.0), vg.Point(12.0,18.0), vg.Point(12,24.0)],
        #[vg.Point(-6.0,4.0), vg.Point(-4.0,6.0), vg.Point(1.0,-3.0), vg.Point(0,-5.0)],
        #External borders
        #[vg.Point(10.0,-5.0), vg.Point(10.0,-10.0), vg.Point(90.0,-10.0), vg.Point(90.0,-5.0)],
        #[vg.Point(10.0,105.0), vg.Point(10.0,110.0), vg.Point(90.0,110.0), vg.Point(90.0,105.0)],
        #[vg.Point(-5.0,10.0), vg.Point(-10.0,10.0), vg.Point(-10.0,90.0), vg.Point(-5.0,90.0)],
        #[vg.Point(105.0,10.0), vg.Point(110.0,10.0), vg.Point(110.0,90.0), vg.Point(105,90.0)]]

def increase_coordinates(polygon, howmuch):
    center_x = 0
    center_y = 0
    for p in polygon:
        center_x += p.x/np.size(polygon)
        center_y += p.y/np.size(polygon)
    for p in polygon:
        angle = math.atan2(p.y-center_y,p.x-center_x)
        #p.x = p.x + np.sign(math.cos(angle))*howmuch
        #p.y = p.y + np.sign(math.sin(angle))*howmuch
        p.x = math.floor(p.x + math.cos(angle)*howmuch)
        p.y = math.floor(p.y + math.sin(angle)*howmuch)

def draw_polygon(polygon, color, size, complete=True):
    local_polygon = [i for i in polygon]

    if complete:
        local_polygon.append(local_polygon[0])
    p1 = local_polygon[0]
    for p2 in local_polygon[1:]:
        plt.plot([p1.x, p2.x], [p1.y, p2.y], color=color, linestyle='solid',  linewidth=size)
        p1 = p2

def draw_visible_vertices(edges, color, size):
    for e in edges:
        plt.plot([e.p1.x, e.p2.x], [e.p1.y, e.p2.y], color=color, linestyle='dashed',  linewidth=size)

def check_point_obstacles_overlap(point,graph,howmuch):
    intersected_polygon = vv.point_in_polygon(point,graph)
    if intersected_polygon != -1:
        return vv.closest_point(point, graph, intersected_polygon, length=howmuch)
    else: return point

def global_path(polys,thymio_position, end_position):

    g = vg.VisGraph()
    g.build(polys)

    # Drawing obstacles as received
    #if len(polys) > 0:
        #for polygon in polys:
            #draw_polygon(polygon,'black',1)

    # Increasing distance from obstacles for starting and ending points
    start_point = check_point_obstacles_overlap(thymio_position,g.graph,65)
    end_point = check_point_obstacles_overlap(end_position,g.graph,65)

    # Increasing obstacle borders
    for p in polys:
        increase_coordinates(p, 60)

    # Checking obstacles overlaping to merge them
    i = 0
    while i < len(polys):
        j = 0
        while j < len(polys):
            if j != i:
                if mo.check_overlap(polys[j],polys[i]):
                    polys[j] = mo.unify(polys[j],polys[i])
                    polys.pop(i)
                    i = 0
                    j = len(polys)
                else:
                    j += 1
            else: j += 1
        i +=1

    # Updating polygons map
    g.build(polys)

    # Drawing the new obstacles (possibly merged)
    if len(polys) > 0:
        for polygon in polys:
            draw_polygon(polygon,'blue',3)

    # Showing the visibility graph
    draw_visible_vertices(g.visgraph.get_edges(),'grey', 1)

    # Computing the shortest path
    shortest_path = g.shortest_path(thymio_position, end_position)
    if len(shortest_path) > 1:
        draw_polygon(shortest_path, 'red', 2, complete=False)

    # Exporting the path sequence (without starting point)
    path = shortest_path.pop(0)
    return path
