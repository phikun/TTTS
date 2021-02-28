#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 用GDAL对人口做掩膜，直接做会出现NoData混淆的问题，所以先读进来原封不动写出去，再切！

# Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
# Date: 2021.01.07

from math import ceil, floor
from osgeo import gdal
import pandas as pd
import geopandas as gpd

# input_raster = "/data/ck/TTTS/Data_structured/Africa/congo-brazzaville/cog_f_10_2020.tif"
# input_shape = "/data/ck/TTTS/Data_structured/Africa/congo-brazzaville/shp/Region-WGS84.shp"
# output_raster = "/data/ck/TTTS/Data_structured/Africa/congo-brazzaville/Population-Mask.tif"
# middle_raster = "/data/ck/TTTS/Data_structured/Africa/congo-brazzaville/Middle.tif"
excel = "./Split-Excels/to-be-calculate.xlsx"
sheet_name = None


def open_excel(file_name, sheet_name=None):
    if sheet_name is None:
        df = pd.read_excel(file_name)
    else:
        df = pd.read_excel(file_name, sheet_name=sheet_name)
    return df


def mask_one_country(input_raster, input_shape, output_raster, middle_raster, offset=2):
    print("Current File: %s" % input_raster, "shape: %s" % input_shape)
    gdal.UseExceptions()

    # Step1: 读进来，按研究区边界切一下, 写出去
    dataset = gdal.Open(input_raster)
    trans = dataset.GetGeoTransform()
    proj = dataset.GetProjection()
    dx, dy = trans[1], trans[5]
    x1, y1 = trans[0], trans[3]
    xsize, ysize = dataset.RasterXSize - 1, dataset.RasterYSize - 1
    
    gdf = gpd.read_file(input_shape)
    shape = gdf["geometry"][0]
    bounds = shape.bounds
    xlb, xrb = min(bounds[0], bounds[2]), max(bounds[0], bounds[2])
    ylb, yrb = min(bounds[1], bounds[3]), max(bounds[1], bounds[3])
    
    sx = max(0, floor((xlb - x1) / dx) - offset)
    ex = min(xsize, ceil((xrb - x1) / dx) + offset)
    sy = max(0, floor((yrb - y1) / dy) - offset)
    ey = min(ysize, ceil((ylb - y1) / dy) + offset)
    
    img = dataset.ReadAsArray(sx, sy, ex - sx + 1, ey - sy + 1)
    sxf = x1 + dx * sx
    syf = y1 + dy * sy
    res_trans = [sxf, dx, 0.0, syf, 0.0, dy] 
    
    nrows, ncols = img.shape
    driver = gdal.GetDriverByName("GTiff")
    outRaster = driver.Create(middle_raster, ncols, nrows, 1, gdal.GDT_Float32)
    outRaster.GetRasterBand(1).WriteArray(img)
    outRaster.SetGeoTransform(res_trans)
    outRaster.SetProjection(proj)
    outRaster.FlushCache()
    outRaster = None
    
    # Step2: 切割写出去的中间文件
    dataset = gdal.Open(middle_raster)
    ds = gdal.Warp(output_raster, dataset, format="GTiff", cutlineDSName=input_shape, cutlineWhere="FIELD = 'whatever'", dstNodata = -999999)
    del ds  # Close Output Data


if __name__ == "__main__":
    df = open_excel(excel, sheet_name)
    input_rasters = df["Population"]
    input_shapes = df["Region"]
    output_rasters = df["Population-Mask"]
    middle_rasters = df["Middle"]

    for (input_raster, input_shape, output_raster, middle_raster) in zip(input_rasters, input_shapes, output_rasters, middle_rasters):
        
        mask_one_country(input_raster, input_shape, output_raster, middle_raster)
        # except Exception:
        #     print("    Failed Treat File: %s" % input_shape)

    print("Finished.")
