#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Travel Tiem to School
# Transfer it to float32, clip output raster by regin shape, and zip it!

# Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
# Date: 2021.01.08

from osgeo import gdal
import pandas as pd

data_file = "/root/data/TTTS/codes/pretreat/Split-Excels/to-be-calculate.xlsx"


def transfer_to_float32(input_file, output_file):
    dataset = gdal.Open(input_file)
    trans = dataset.GetGeoTransform()
    proj = dataset.GetProjection()
    
    img = dataset.ReadAsArray().astype("float32")
    
    nrows, ncols = img.shape
    print("Output shape:", img.shape)
    driver = gdal.GetDriverByName("GTiff")
    outRaster = driver.Create(output_file, ncols, nrows, 1, gdal.GDT_Float32)
    outRaster.GetRasterBand(1).WriteArray(img)
    outRaster.SetGeoTransform(trans)
    outRaster.SetProjection(proj)
    outRaster.FlushCache()
    outRaster = None


def zip_raster(input_raster, ouput_raster, method="LZW"):
    dataset = gdal.Open(input_raster)
    driver = gdal.GetDriverByName('GTiff')
    driver.CreateCopy(ouput_raster, dataset, strict=1, options=["BIGTIFF=YES", "TILED=YES", "COMPRESS={0}".format(method)])
    del dataset


def clip_raster_by_shape(input_raster, input_shape, output_raster):
    dataset = gdal.Open(input_raster)
    ds = gdal.Warp(output_raster, dataset, format="GTiff", cutlineDSName=input_shape, cutlineWhere="FIELD = 'whatever'", dstNodata = -3.402823466e+38)
    del ds  # Close Output Data
    zip_raster(output_raster, output_raster)


if __name__ == "__main__":
    df = pd.read_excel(data_file)
    outputs, clips, regions, middles = \
        df["output"], df["output-Clip"], df["Region"], df["Middle"]
        
    for (output, clip, region, middle) in zip(outputs, clips, regions, middles):
        print("Current File: %s" % output)
        try:
            transfer_to_float32(output, middle)
            clip_raster_by_shape(middle, region, clip)
        except Exception:
            print("    Failed Treat: %s" % output)
    
    print("Finished.")
