from shapely.geometry import Polygon
from shapely.ops import unary_union
import pyvisgraph as vg
import numpy as np

# Merge obstacles
def unify(p1,p2):
    if p1 == []:
        return p2
    elif p2 == []:
        return p1
    else:
        polygon1 = vg2shapely(p1)
        polygon2 = vg2shapely(p2)
        polygons = [polygon1, polygon2]
        print(polygons)
        u = unary_union(polygons)
        return shapely2vg(u)

# Checks obstacles overlap
def check_overlap(p1,p2):
    polygon1 = vg2shapely(p1)
    polygon2 = vg2shapely(p2)
    return polygon1.intersects(polygon2)

# Conversion function
def vg2shapely(vg_polygon):
    vect = []
    for p in vg_polygon:
        vect.append((p.x,p.y))
    return Polygon(vect)

# Conversion function
def shapely2vg(shapely_polygon):
    vg_polygon = []
    for p in range(0,int((np.size(shapely_polygon.exterior.coords)-2)/2)):
        x,y = shapely_polygon.exterior.coords[p]
        vg_polygon.append(vg.Point(x,y))
    return vg_polygon
