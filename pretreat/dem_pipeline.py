#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# TTTS: Treat DEM Pipeline
#   Given region shapefile and population raster, merge and resample DEM to output file.
#   This script transfer given data to [DEM-Slope]! 
#   REMEMBER call 'pop-mask.py' FIRST!  [Do it Tommorrow]
#   Then calling 'slope-resample.py' to get walk speed!

# Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
# Date: 2021.01.31

from math import floor
from osgeo import gdal
from osgeo.gdal import gdalconst
import geopandas as gpd
import pandas as pd
import os

data_excel = "./Split-Excels/to-be-calculate.xlsx"
sheet_name = None


def open_excel(file_name, sheet_name=None):
    if sheet_name is None:
        df = pd.read_excel(file_name)
    else:
        df = pd.read_excel(file_name, sheet_name=sheet_name)
    return df


def compress(path, target_path, method="LZW"):
    dataset = gdal.Open(path)
    driver = gdal.GetDriverByName('GTiff')
    driver.CreateCopy(target_path, dataset, strict=1, options=["TILED=YES", "COMPRESS={0}".format(method)])
    del dataset


# Get row & column number in SRTM DEM, from given longitude and laititude
# If region shape is out of range, just return negative row number!
# Return values: ((col, lng_start), (row, lat_start))
def get_row_col_number_from_lng_lat(lng, lat):
    lng0, lat0 = -180, 60
    dlng, dlat = 5, -5
    
    col = floor((lng - lng0) / dlng) + 1
    lng_start = lng0 + dlng * (col - 1)
    
    row = floor((lat - lat0) / dlat) + 1
    lat_start = lat0 + dlat * (row - 1)
        
    return ((col, lng_start), (row, lat_start))


# Merge DEM from splited .img files. If the file isn't exist, print error message
def merge_dem_from_region(region_shp, output_file):
    n_block_row, n_block_col = 6000, 6000
    
    gdf = gpd.read_file(region_shp)
    region = gdf["geometry"][0]
    envelope = region.envelope.boundary
    (xs, ys) = envelope.xy
    
    left_upper_x, left_upper_y = min(xs), max(ys)
    ((start_col, start_lng), (start_row, start_lat)) = get_row_col_number_from_lng_lat(left_upper_x, left_upper_y)
    right_lower_x, right_lower_y = max(xs), min(ys)
    ((end_col, end_lng), (end_row, end_lat)) = get_row_col_number_from_lng_lat(right_lower_x, right_lower_y)

    # Then Write to GeoTiff
    n_cols = (end_col - start_col + 1) * n_block_col
    n_rows = (end_row - start_row + 1) * n_block_row
    driver = gdal.GetDriverByName("GTiff")
    out_raster = driver.Create(output_file, n_cols, n_rows, 1, gdal.GDT_Int16)
    out_raster.SetGeoTransform([start_lng, 5 / 6000, 0, start_lat, 0, -5 / 6000])
    out_raster.SetProjection('GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]]')
    out_raster.GetRasterBand(1).SetNoDataValue(-32768)
    
    for (i, row) in enumerate(range(start_row, end_row + 1)):
        for (j, col) in enumerate(range(start_col, end_col + 1)):
            col_offset = j * n_block_col
            row_offset = i * n_block_row
            data = "srtm_%02d_%02d" % (col, row)
            
            tiff_name = "/root/data/TTTS/SRTM/" + data + ".img/" + data + ".tif"
            if not os.path.isfile(tiff_name):
                print("Cannot Find File: %s" % tiff_name)
                continue
            zone = gdal.Open(tiff_name)
            arr = zone.ReadAsArray()
            out_raster.GetRasterBand(1).WriteArray(arr, col_offset, row_offset)
            del zone
    
    out_raster.FlushCache()
    del out_raster
    compress(output_file, output_file)
    print("Finished Merge %s" % region_shp)


# Merge DEM from GeoTIFF, using in 180E regions
def merge_dem_from_tiff(input_tif, output_file):
    n_block_row, n_block_col = 6000, 6000
    
    inp_ds = gdal.Open(input_tif, gdal.GA_ReadOnly)
    inp_trans = inp_ds.GetGeoTransform()
    xsize = inp_ds.RasterXSize
    ysize = inp_ds.RasterYSize
    
    (dx, dy) = (inp_trans[1], inp_trans[5])
    (left_upper_x, left_upper_y) = (inp_trans[0], inp_trans[3])
    right_lower_x = left_upper_x + dx * xsize
    right_lower_y = left_upper_y + dy * ysize
    
    ((start_col, start_lng), (start_row, start_lat)) = get_row_col_number_from_lng_lat(left_upper_x, left_upper_y)
    ((end_col, end_lng), (end_row, end_lat)) = get_row_col_number_from_lng_lat(right_lower_x, right_lower_y)

    # Then Write to GeoTiff
    n_cols = (end_col - start_col + 1) * n_block_col
    n_rows = (end_row - start_row + 1) * n_block_row
    driver = gdal.GetDriverByName("GTiff")
    out_raster = driver.Create(output_file, n_cols, n_rows, 1, gdal.GDT_Int16)
    out_raster.SetGeoTransform([start_lng, 5 / 6000, 0, start_lat, 0, -5 / 6000])
    out_raster.SetProjection('GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]]')
    out_raster.GetRasterBand(1).SetNoDataValue(-32768)
    
    for (i, row) in enumerate(range(start_row, end_row + 1)):
        for (j, col) in enumerate(range(start_col, end_col + 1)):
            col_offset = j * n_block_col
            row_offset = i * n_block_row
            data = "srtm_%02d_%02d" % (col, row)
            
            tiff_name = "/root/data/TTTS/SRTM/" + data + ".img/" + data + ".tif"
            if not os.path.isfile(tiff_name):
                print("Cannot Find File: %s" % tiff_name)
                continue
            zone = gdal.Open(tiff_name)
            arr = zone.ReadAsArray()
            out_raster.GetRasterBand(1).WriteArray(arr, col_offset, row_offset)
            del zone
    
    out_raster.FlushCache()
    del out_raster
    compress(output_file, output_file)
    print("Finished Merge %s" % input_tif)
    

# Resample DEM values as the size of WorldPop, using Bi-linear Method
def dem_resample(input_dem, pop_raster, output_file):
    gdal.UseExceptions()
    dem = gdal.Open(input_dem, gdal.GA_ReadOnly)
    dem_proj = dem.GetProjection()

    pop = gdal.Open(pop_raster, gdal.GA_ReadOnly)
    pop_proj = pop.GetProjection()
    pop_trans = pop.GetGeoTransform()
    pop_band = pop.GetRasterBand(1)
    pop_width = pop.RasterXSize
    pop_height = pop.RasterYSize
    pop_nbands = pop.RasterCount

    driver = gdal.GetDriverByName('GTiff')
    output = driver.Create(output_file, pop_width, pop_height, pop_nbands, pop_band.DataType)
    output.SetGeoTransform(pop_trans)
    output.SetProjection(pop_proj)

    gdal.ReprojectImage(dem, output, dem_proj, pop_proj, gdalconst.GRA_Bilinear, 0.0, 0.0,)
    # del res_ds
    # compress(output_file, output_file)


# Calculate slope
def calculate_slope(input_file, output_file):
    dem = gdal.Open(input_file, gdal.GA_ReadOnly)
    res = gdal.DEMProcessing(output_file, dem, "slope", format="GTiff", band=1, scale=111120, slopeFormat="degree")
    del res
    compress(output_file, output_file)


# Pipeline
def dem_pipeline(region_shp, pop_raster, middle_file, dem_file, slope_file):
    merge_dem_from_region(region_shp, middle_file)
    dem_resample(middle_file, pop_raster, dem_file)
    compress(dem_file, dem_file)
    calculate_slope(dem_file, slope_file)
    # print("Finished %s" % region_shp, end="\n\n")


if __name__ == "__main__":
    print("Hello World!")

    # df = pd.read_excel(data_excel)
    df = open_excel(data_excel, sheet_name)
    names, regions, pops, middles, dems, slopes = df["Name"], df["Region"], df["Population-Mask"], df["Middle"], df["DEM"], df["DEM-Slope"]
    
    for (name, region, pop, middle, dem, slope) in zip(names, regions, pops, middles, dems, slopes):
        print("Current Region: %s" % name)
        
        try:
            dem_pipeline(region, pop, middle, dem, slope)
            print("Finished: %s" % name, end="\n\n")
        except Exception:
            print("    Failed Treat Region: %s" % name)

    # region_shp = "/data/ck/Education/mapping-with-dem/OSM-Version/North-America/Canada/British-Columbia/Region-WGS84.shp"
    # pop_raster = "/data/ck/Education/mapping-with-dem/OSM-Version/North-America/Canada/British-Columbia/Population-Clip.tif"
    # middle_file = "/data/ck/Education/mapping-with-dem/OSM-Version/North-America/Canada/British-Columbia/Middle.tif"
    # output_file = "/data/ck/Education/mapping-with-dem/OSM-Version/North-America/Canada/British-Columbia/DEM-Cut.tif"
    # slope_file = "/data/ck/Education/mapping-with-dem/OSM-Version/North-America/Canada/British-Columbia/DEM-Slope.tif"

    # dem_pipeline(region_shp, pop_raster, middle_file, output_file)
    # calculate_slope(output_file, slope_file)
