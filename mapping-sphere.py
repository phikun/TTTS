# -*- coding: utf-8 -*-
# 上学时间制图：用Python直接掉Dijkstra和计算几何的C++动态链接库
# 主要分成3部分＿
#     Step1: 根据路多省网数据和学校位置建图（需事先裁剪路网and校正学校的几何位置）
#     Step2: Dijkstra算法求道路结点上学的最短时闿
#     Step3: 根据制图中心点的经纬度求栅格中心点的上学时间（需事先生成栅格中心点并投影到Albers105坐标系下＿
# 2020.10.31更新：允许道路shapefile中存在speed属性，并按速度计算道路通行时间
# 2020.11.02更新：把一个省的处理弄成函数，便于此后循环
# 2020.11.10更新：在调用C++程序时把Python数据类型转成了C风格数组
# 2020.11.16更新：修改数据输入方式，把投影后的栅格改成WGS84下的栅格，在程序内部投影到Albers105
# 2020.11.17更新：用NumPy计算栅格中心点的经纬度
# 2020.11.27更新：直接用经纬度计算上学时间

# Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
# Date: 2020.11.27

import utility as util

from osgeo import gdal
import rasterio as rio
from rasterio.mask import mask
from shapely import geometry as geo
from shapely.ops import nearest_points
import geopandas as gpd
import numpy as np
import pandas as pd

from time import perf_counter  # 计算程序运行时间

import ctypes

cll1 = ctypes.cdll.LoadLibrary("./insert_school_sphere.so")
cll2 = ctypes.cdll.LoadLibrary("./dijkstra_sphere.so")
cll3 = ctypes.cdll.LoadLibrary("./travel_time_sphere.so")

road_prefix, road_suffix = "./data/roads/", "道路-WGS84-Speed.shp"
school_prefix, school_suffix = "./data/schools/primary/", "小学-WGS84.shp"
raster_prefix, raster_suffix = "./data/population/", "人口-Clip.tif"
output_prefix, output_suffix = "./results/小学/", "-小学-Full-Sphere-Box100-Dist.tif"
input_file = "./data/input.xlsx"

maxn = 6E6  # C++程序中数组大小，若点数太多需要修改C++程序重新编译


# 在顶点表和边表中插入学校
# Inputs: schools_points -> 带插入的学校几何形状列表，需要是gpd.geoseries.GeoSeries类型
#         其他是原始的顶点表和边表索引
# Outputs: schoolIndexes -> 学校结点在顶点集中的编号
#          其他是更新后的顶点表和边表索引
def insert_schools_into_graph(school_points, index2edge, edge2index, index2vertex, vertex2index):
    assert type(school_points) == gpd.geoseries.GeoSeries
    
    # 调用C++函数完成学校插入
    # Step1: 生成C++函数需要的输入: schools: [(lng, lat)], seg_idx: [i1, i2],  seg_coords[(lng1, lat1, lng2, lat2)]
    schools = ctypes.c_double * (2 * len(school_points))
    schools = schools()
    cnt = 0
    for point in school_points:
        schools[cnt] = point.x 
        schools[cnt + 1] = point.y
        cnt += 2
        
    seg_idx = list(index2edge.keys())
    
    seg_coords = ctypes.c_double * (4 * len(seg_idx))
    seg_coords = seg_coords()
    cnt = 0
    for idx in seg_idx:
        edge = index2edge[idx]
        x1, y1 = index2vertex[edge[0][0]]
        x2, y2 = index2vertex[edge[0][1]]
        seg_coords[cnt] = x1;     seg_coords[cnt + 1] = y1
        seg_coords[cnt + 2] = x2; seg_coords[cnt + 3] = y2
        cnt += 4
    
    seg_indexes = ctypes.c_int32 * len(seg_idx)
    seg_indexes = seg_indexes()
    for (i, seg) in enumerate(seg_idx):
        seg_indexes[i] = seg
    
    # Step2: 调用C++程序，空间索引找到最临近线段
    st = perf_counter()
    cll1.find_nearest_segment.restype = ctypes.POINTER(ctypes.c_int32)
    npidx = cll1.find_nearest_segment(len(school_points), schools, len(seg_idx), seg_indexes, seg_coords)
    npidxes = [npidx[i] for i in range(len(seg_idx))]
    cll1.dispose(npidx)  # prevent MEMORY LEAK!
    del npidx
    et = perf_counter()
    print("Index Time:", et - st)
    
    # Step3: 更新顶点表和边表，插入新的学校点、插入学校点到两侧顶点的边
    vertidx = len(index2vertex) + 1
    edgeidx = len(index2edge) + 1
    schoolIndexes = []  # 路网上学校的列表
    
    for (point, sid) in zip(school_points, npidxes):
        lng, lat = point.x, point.y
        assert (lng, lat) not in vertex2index.keys()  # 假定学校不与道路结点重合，出问题再说
        
        roadSeg = geo.LineString([index2vertex[index2edge[sid][0][0]], index2vertex[index2edge[sid][0][1]]])
        vertPoint = nearest_points(point, roadSeg)[1]  # shapely的文档说最短线段的顺序与输入顺序相同，则第2个是路网上的炿
        vlng, vlat = vertPoint.x, vertPoint.y
        
        if (vlng, vlat) not in vertex2index.keys(): # 若垂点在不在路网结点集合中，加点加边
            p0 = (vlng, vlat)
            vertex2index[p0] = vertidx
            index2vertex[vertidx] = p0
            vertidx += 1
            
            p1, p2 = index2vertex[index2edge[sid][0][0]], index2vertex[index2edge[sid][0][1]]
            orispeed = index2edge[sid][1]
            edge2index[(vertex2index[p0], vertex2index[p1])] = edgeidx
            index2edge[edgeidx] = ((vertex2index[p0], vertex2index[p1]), orispeed)
            edgeidx += 1
            edge2index[(vertex2index[p0], vertex2index[p2])] = edgeidx
            index2edge[edgeidx] = ((vertex2index[p0], vertex2index[p2]), orispeed)
            edgeidx += 1
            
        schoolIndexes.append(vertex2index[vlng, vlat])  # 把学校点记下板
        
    return (schoolIndexes, index2edge, edge2index, index2vertex, vertex2index)


# 导入栅格中心点矢量文件，矢量文件必须有Rows、Cols列表明这一点的行列号，【这个方法太慢〿
# 2020.10.31更新：不判断点是否在多边形中，先生成面状的，再用shapefile掩膜；取消了polygon_file参数
# Inputs: point_file -> 栅格中心点矢量文件，含有Rows、Cols列，可以用utility.build_raster_center_point函数生成并在ArcGIS中投彿
# Outputs: mat -> 生成的全为NaN的栅栿
#          lst -> 待求栅格中心点列表[(row, col, lng, lat)]，用于C++计算该点上学最短时闿
#          min_lng, lax_lat -> 最小经度和最大纬度，用于最后GDAL写GeoTiff
def import_raster_center_point(point_file):
    point_df = gpd.read_file(point_file, encoding="UTF-8")
    points = point_df["geometry"]
    rows, cols = point_df["Rows"], point_df["Cols"]
    nrows, ncols = int(max(rows) + 1), int(max(cols) + 1)  # 获取行列敿
    min_lng, max_lat = min(point_df["lng"]), max(point_df["lat"])
    
    mat = np.ones([nrows, ncols])
    mat.fill(np.nan)
    
    lst = []
    for (row, col, point) in zip(map(int, point_df["Rows"]), map(int, point_df["Cols"]), points):
        lng, lat = point.x, point.y
        lst.append((row, col, lng, lat))
    
    return (mat, lst, min_lng, max_lat)


# 用投影后的栅格获取栅格中心点，import_raster_center_point的替代函敿
# 返回空矩阵、用于C++制图函数输入的列表、变换矩阵、投影方法
# 2020.11.10更新：把行列号和栅格中心点坐标拆房个列表，并返回待求栅格中心点个数
# 2020.11.27更新：不投影，直接按经纬度返回
def construct_raster_center_list(raster_file):    
    dataset = gdal.Open(raster_file)
    trans = dataset.GetGeoTransform()
    proj = dataset.GetProjection()
    mat = dataset.ReadAsArray()
    rows, cols = np.where(mat > -100000)  # 查询在研究区内的栅格行列号, 无人口的栅格值是-99999, 不在研究区内的NoData值是-INF
    
    n_center_points = len(rows)
    rowcols, coords = [], ctypes.c_double * (2 * n_center_points)
    coords = coords()
    
    xoffset = trans[1] / 2.0; yoffset = trans[5] / 2.0
    lngs = list(trans[0] + cols * trans[1] + rows * trans[2] + xoffset)
    lats = list(trans[3] + cols * trans[4] + rows * trans[5] + yoffset)
    
    ptr = 0
    for (row, col, lng, lat) in zip(rows, cols, lngs, lats):
        coords[ptr] = lng; coords[ptr + 1] = lat
        rowcols.append((row, col))
        ptr += 2
    
    res_mat = np.ones(mat.shape)
    res_mat.fill(np.nan)
    return (res_mat, n_center_points, rowcols, coords, trans, proj)


# 把计算结果写入GeoTiff文件
# 2020.11.01更新：把输入min_lng、max_lat、dx、dy改成输入栅格投影信息和变换矩阿
# Inputs: file_name -> 输出文件吿
#             ttlst -> 调用travel_time的C++库返回的栅格中心点上学时间列衿
#  min_lng, max_lat -> 最小经度和最大纬度（import_raster_center_point函数返回＿
#            dx, dy -> 栅格在经度和纬度方向上的分辨率（dy是负数），按理由可以直接在生成点时确宿
# def wirte_to_geotiff(file_name, mat, ttlst, min_lng, max_lat, dx, dy):
def wirte_to_geotiff(file_name, mat, ttlst, trans, proj):
    # 填写结果矩阵
    for tup in ttlst:
        r, c, dist = tup
        mat[r, c] = dist
    # mat = np.flipud(mat)
    nrows, ncols = mat.shape
    
    # 投影是WGS84
    # proj = 'GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563,AUTHORITY["EPSG","7030"]],AUTHORITY["EPSG","6326"]],PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433,AUTHORITY["EPSG","9122"]],AXIS["Latitude",NORTH],AXIS["Longitude",EAST],AUTHORITY["EPSG","4326"]]'
    # trans = [min_lng, dx, 0.0, max_lat, 0.0, dy]

    driver = gdal.GetDriverByName('Gtiff')
    outRaster = driver.Create(file_name, ncols, nrows, 1, gdal.GDT_Float64)  # 1 -> 1个波殿
    outband = outRaster.GetRasterBand(1)
    outband.WriteArray(mat)
    outRaster.SetGeoTransform(trans)
    outRaster.SetProjection(proj)
    outRaster.FlushCache()


# 按照几何形状裁剪栅格；rasterio。mask可能有问题，出来之后会错位；ArcGIS的mask正确
# 所以先把矩阵的结果写入GeoTiff，之后再用ArcGIS按掩膜提叿
# Inputs: polygon_file -> 研究区的.shp文件
#           input_file -> 输入待裁剪的文件
#          output_file -> 输出裁剪后的文件
def mask_by_shape(polygon_file, input_file, output_file):
    shape = gpd.read_file(polygon_file, encoding="UTF-8")["geometry"][0]
    
    input_ = rio.open(input_file)
    features = [shape.__geo_interface__]
    res_image, res_trans = mask(input_, features, crop=True, nodata=input_.nodata)
    res_meta = input_.meta.copy()
    res_meta.update({"driver": "GTiff",
                     "height": res_image.shape[1],
                     "weight": res_image.shape[2],
                     "transform": res_trans})
    res = rio.open(output_file, "w", **res_meta)
    res.write(res_image)


# 给行一个省的基本数据、为该省制图
def process_one_province(road_file, school_file, raster_file, output_file):
    start_time = perf_counter()
    
    # 读入道路和学校数捿
    road = gpd.read_file(road_file)
    roads, speeds = road["geometry"], road["speed"]
    school = gpd.read_file(school_file)
    schools = school["geometry"]
    
    global vertex2index, index2vertex, edge2index, index2edge
    
    # 建立顶点表和边表
    vertex2index, index2vertex = util.build_vertex_table(roads)
    edge2index, index2edge = util.build_edge_table(roads, speeds, vertex2index) 
    
    if len(index2edge) > maxn:  # 如果线段太多需要报错
        raise RuntimeError("Too many edges!, len(edges) = %d!" % len(index2edge))
    
    school_indexes, index2edge, edge2index, index2vertex, vertex2index = \
        insert_schools_into_graph(schools, index2edge, edge2index, index2vertex, vertex2index)
    
    global vertex_time
    
    schools = ctypes.c_int32 * len(school_indexes)
    schools = schools()
    for i in range(len(school_indexes)):
        schools[i] = school_indexes[i]
    
    # Dijkstra算法求最短路
    edges, speeds, vidxes, vcoords = util.build_edge_tuples_for_dijkstra(index2edge, index2vertex)
    dst = perf_counter()
    cll2.dijkstra.restype = ctypes.POINTER(ctypes.c_double)
    vt = cll2.dijkstra(len(index2vertex), len(index2edge), len(school_indexes), schools, edges, speeds, vidxes, vcoords)
    vertex_time = {}
    for id_ in range(1, len(index2vertex) + 1):
        vertex_time[id_] = vt[id_]
    cll2.dispose(vt)  # prevent MEMORY LEAK!
    del vt
    det = perf_counter()
    print("Dijkstra Time:", det - dst)
    
    # 再检查一下到Dijkstra算法的正确性: 数值合理，但和投影坐标系的差距较大，一会制图看一下！
    
    
    # 调用C++的空间索引制囿
    # 创建C风格的栅格中心点数组
    mat, n_center_points, rowcols, center_coords, trans, proj = construct_raster_center_list(raster_file)
    
    # 创建C风格的道路结点数组
    n_vertex = len(index2vertex)
    ids, coords, times = ctypes.c_int32 * n_vertex, ctypes.c_double * (2 * n_vertex), ctypes.c_double * n_vertex
    ids, coords, times = ids(), coords(), times()
    for (ptr, key) in enumerate(index2vertex.keys()):
        ids[ptr] = key
        times[ptr] = vertex_time[key]
        coords[ptr << 1] = index2vertex[key][0]
        coords[ptr << 1 | 1] = index2vertex[key][1]
    
    # 创建C风格的线串数组
    n_edges = len(index2edge)
    verts, speeds = ctypes.c_int32 * (2 * n_edges), ctypes.c_double * n_edges
    verts, speeds = verts(), speeds()
    for (ptr, key) in enumerate(index2edge.keys()):
        speeds[ptr] = index2edge[key][1]
        verts[ptr << 1] = index2edge[key][0][0]
        verts[ptr << 1 | 1] = index2edge[key][0][1]
    
    mst = perf_counter()
    cll3.travel_time.restype = ctypes.POINTER(ctypes.c_double)
    c_ttlst = cll3.travel_time(n_vertex, ids, coords, times, n_edges, verts, speeds, n_center_points, center_coords)
    ttlst = [(rowcols[i][0], rowcols[i][1], c_ttlst[i]) for i in range(n_center_points)]
    cll3.dispose(c_ttlst)  # prevent MEMORY LEAK
    del c_ttlst
    met = perf_counter()
    print("Mapping Time:", met - mst)
    
    # 最后保存到文件, Here [rowcols] weill be used
    wirte_to_geotiff(output_file, mat, ttlst, trans, proj)
    
    end_time = perf_counter()
    
    print("Finished.")
    print("Total Time =", end_time - start_time)
    

# 根据省名、AdminCode、分辨率生成输入输出数据的文件名，并调用process_one_province函数完成计算
def treat_one_province_call(tup):
    adcode, name, resolution = tup
    road_file = road_prefix + name + road_suffix
    school_file = school_prefix + name + school_suffix
    raster_file = raster_prefix + name + raster_suffix
    output_file = output_prefix + name + output_suffix
    process_one_province(road_file, school_file, raster_file, output_file)
    
    res_str = "Finished: " + name + "\n"
    print(res_str, end="")


if __name__ == "__main__":
    df = pd.read_excel(input_file)
    
    input_ = list(zip(df["adcode"], df["name"], df["resolution"]))
    for tup in input_:
        treat_one_province_call(tup)
    
    print("Finished.")
