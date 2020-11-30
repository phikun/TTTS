# -*- coding: utf-8 -*-
# 上学时间制图：直接用经纬度制图的mapping-sphere脚本中的常用函数，防止单个文件过大
# 包括：1. 构造顶点表: build_vertex_table
#       2. 根据LineString查询顶点表添加弧段: update_edge_list（构造边表的辅助函数）
#       3. 构造边表: build_edge_table
#       4. 为Dijkstra算法构造输入数据

# Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
# Date: 2020.11.27

from shapely import geometry as geo
import geopandas as gpd
import ctypes


# 构造顶点表
# Inputs: shapes -> GeoDataFrame的geometry列，必须是gpd.geoseries.GeoSeries类型
# Outputs: vertex2index -> 顶点xy坐标到编号的字典，用于构造边衿
#          index2vertex -> 编号到顶点xy的字典，作为网络结点的几何标访
def build_vertex_table(shapes):
    assert type(shapes) == gpd.geoseries.GeoSeries
    
    points = []
    for shp in shapes:
        if type(shp) == geo.multilinestring.MultiLineString:  # 多线段，需要遍历处理各子线殿
            for arc in shp:
                lngs, lats = arc.xy
                for (lng, lat) in zip(lngs, lats):
                    points.append((lng, lat))
        else:  # 单线殿
            lngs, lats = shp.xy
            for (lng, lat) in zip(lngs, lats):
                points.append((lng, lat))

    point_set = set(points)  # 用集合去掉重复点
    vertex2index, index2vertex = {}, {}
    idx = 1
    for point in point_set:
        vertex2index[point] = idx
        index2vertex[idx] = point
        idx += 1

    return (vertex2index, index2vertex)


# 给定一个shapely.geometry.LineString，查询顶点表更新弧段
# Inputs:    arc -> LineString型的弧段
#   vertex2index -> 弧段结点xy坐标到结点编号的字典
#          edges -> 待更新的边表
#          speed -> 这条边的速度
# Outputs: edges -> 更新后的边表
def update_edge_list(arc, vertex2index, edges, speed):
    assert type(arc) == geo.linestring.LineString
    
    lng, lat = arc.xy
    size = len(lng)
    for i in range(1, size):
        p1 = vertex2index[(lng[i - 1], lat[i - 1])]
        p2 = vertex2index[(lng[i], lat[i])]
        if p1 != p2:
            edges.append(((p1, p2), speed))
    
    return edges


# 构造边表，这里假定每条路都是双向的，【需要注意重边和自环的情况〿
# 注意：这里构造的边表是【无向】，即【只标记每条线段的始点和终点】，最后生成Dijkstra算法输入数据时记得两边都加边＿
# 数据里可能用重边的情况，插入两次就行了
# Inputs: shapes -> GeoDataFrame的geometry列，必须是gpd.geoseries.GeoSeries类型
#         speeds -> 与shapes对应的通行速度刿
#   vertex2index -> 构造顶点表时生成的、弧段结点xy到结点编号的字典
# Outputs: edge2index -> 每条边到边号的字典（似乎没啥用，先输出好了）
#          index2edge -> 边号到边两端端点编号的字典（似乎没啥用，先输出好了），{idx: (start, end), speed}＿新加了速度属怿
def build_edge_table(shapes, speeds, vertex2index):
    assert type(shapes) == gpd.geoseries.GeoSeries
    assert type(vertex2index) == dict
    
    edges = []
    for (shp, speed) in zip(shapes, speeds):
        if type(shp) == geo.multilinestring.MultiLineString:  # 多线段，需要遍历处理各子线殿
            for arc in shp:
                edges = update_edge_list(arc, vertex2index, edges, speed)
        else:  # 单线殿
            edges = update_edge_list(shp, vertex2index, edges, speed)
    
    edge2index, index2edge = {}, {}
    for (idx, edge) in enumerate(edges, 1):
        edge2index[edge[0]] = idx  # 边到id号的索引中无速度属怿
        index2edge[idx] = edge     # id号到边的索引中有速度属怿
    
    return (edge2index, index2edge)


# 根据道路的通行速度、根据边表构造用于Dijkstra算法输入的边的元组的列表
# 2020.11.09更新：为适合C++程序输入，将带速度的边表拆成边表和速度表
# 2020.11.27更新：直接把速度传到C++程序中，并在C++程序中用地理坐标系计算通行时间
def build_edge_tuples_for_dijkstra(index2edge, index2vertex):
    n_edge = len(index2edge)
    edges = ctypes.c_int32 * (2 * n_edge)
    edges = edges()
    speeds = ctypes.c_double * n_edge
    speeds = speeds()
    
    for (idx, key) in enumerate(index2edge.keys()):
        speed = index2edge[key][1]
        speeds[idx] = speed
        edges[idx << 1] = index2edge[key][0][0]
        edges[idx << 1 | 1] = index2edge[key][0][1]
    
    # And build vertex table
    n_vertex = len(index2vertex.keys())
    vidxes, vcoords = ctypes.c_int32 * (n_vertex), ctypes.c_double * (2 * n_vertex)
    vidxes, vcoords = vidxes(), vcoords()
    
    for (idx, key) in enumerate(index2vertex.keys()):
        vidxes[idx] = key
        vcoords[idx << 1] = index2vertex[key][0]
        vcoords[idx << 1 | 1] = index2vertex[key][1]
    
    return (edges, speeds, vidxes, vcoords)
