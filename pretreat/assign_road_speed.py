# -*- coding: utf-8 -*-
# 上学时间制图：给道路类别赋予速度属性
# 根据dwh课上发的中国区域Open Street Map道路速度表
# Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
# Date: 2020.10.31

import pandas as pd
import geopandas as gpd

# road_file = "./data/shapefiles/37-山东/山东省道路-Albers105.shp"  # 道路shp文件，需要有fclass字段
speed_file = "./OSM-RoadSpeed.xlsx"  # 速度分类文档，需要有ename、cname、speed字段，且速度单位是km/h
# output_file = "./data/shapefiles/37-山东/山东省道路-Speed-Albers105.shp"  # 输出文件

# provs = ["云南省", "西藏自治区", "甘肃省", "青海省", "新疆维吾尔自治区"]
# input_prefix = "./data/shapefiles/roads/Albers105/"
# output_prefix = "./data/shapefiles/roads/Albers105-Speed/"


# 建立OSM道路分类到中文名称和速度的列表
# Inputs: df -> OSM道路分类表格，要求有ename、cname、speed三列，分别表示英文名、中文名、速度(km/h)
def build_road_index(df):
    e2cname, e2speed = {}, {}
    enames, cnames, speeds = df["ename"], df["cname"], df["speed"]
    for (ename, cname, speed) in zip(enames, cnames, speeds):
        e2cname[ename] = cname
        e2speed[ename] = speed * 1000 / 60  # 把km/h换成m/min
    return e2cname, e2speed


# 给道路的shp添加中文类别名称和速度属性（cname、speed）
# Inputs: gdf -> 待添加速度属性的GeoDataFrame，需要有fclass列
#         e2cname, e2speed -> 由build_road_index函数建立的类别到中文名称和速度(m/min)的映射
# Outputs: 更新后的gdf
def assign_road_speed(gdf, e2cname, e2speed):
    columns = set(gdf.columns)  # Acquire fclass column first
    fclass = "fclass"
    if fclass not in columns and "type" in columns:
        fclass = "type"
    if fclass not in columns and "highway" in columns:
        fclass = "highway"

    cnames, speeds = [], []
    for fclass in gdf[fclass]:  # Some files have 'type' column name!
        if fclass in e2cname.keys():  # 若道路类别在给定表中，按其真实类别赋值
            cnames.append(e2cname[fclass])
            speeds.append(e2speed[fclass])
        else:                         # 若不在给定表中，按未分类道路赋值
            cnames.append("未知道路类型")
            speeds.append(e2speed["unclassified"])
    # tmp = pd.DataFrame({"cname": cnames, "speed": speeds})
    tmp = pd.DataFrame({"speed": speeds})  # Chinese Name is useless!
    res = gdf.join(tmp)
    return res


# 给一个省的道路赋值
def assgine_one_province(e2cname, e2speed, road_file, output_file):
    gdf = gpd.read_file(road_file, encoding="UTF-8")
    res = assign_road_speed(gdf, e2cname, e2speed)
    res.to_file(output_file, encoding="UTF-8")


if __name__ == "__main__":
    df = pd.read_excel(speed_file)
    e2cname, e2speed = build_road_index(df)
    
    # road_file = "../OSM-Version/North-America/USA/New Mexico/Road-WGS84.shp"
    # output_file= "../OSM-Version/North-America/USA/New Mexico/Road-WGS84-Speed.shp"
    
    df1 = pd.read_excel("../180E-check/check.xlsx")
    road_files, output_files = df1["Road"], df1["Road-Speed"]
    for (road_file, output_file) in zip(road_files, output_files):
        print("Current File: %s" % road_file)
    
        assgine_one_province(e2cname, e2speed, road_file, output_file)
    
    print("Finished.")
    
    # assgine_one_province(e2cname, e2speed, "./data/shapefiles/roads/Albers105/澳门特别行政区道路.shp", "./data/shapefiles/roads/Albers105-Speed/澳门特别行政区道路-Speed.shp")
    
    """
    for prov in provs:
        print("Current Province", prov)
        input_file = input_prefix + prov + "全域道路.shp"
        output_file = output_prefix + prov + "道路-Speed.shp"
        assgine_one_province(e2cname, e2speed, input_file, output_file)
    
    print("Finished.") """
    
    """ 
    gdf = gpd.read_file(road_file, encoding="UTF-8")
    df = pd.read_excel(speed_file)
    
    e2cname, e2speed = build_road_index(df)
    res = assign_road_speed(gdf, e2cname, e2speed)
    res.to_file(output_file, encoding="UTF-8") """
    
    
