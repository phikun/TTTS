# Global Map of Travel Time to School
#   Resample Slope to Walk Speed:
#      0 -  3 -> 4.0 km/h
#      3 -  8 -> 3.5 km/h
#      8 - 15 -> 3.0 km/h
#     15 - 25 -> 2.5 km/h
#     25 - 35 -> 2.0 km/h
#     35 - 45 -> 1.5 km/h
#        > 45 -> 1.0 km/h

# Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
# Date: 2021.02.04

from osgeo import gdal
import pandas as pd
import numpy as np

fname = "./Split-Excels/to-be-calculate.xlsx"
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


def slope_resample(input_name, output_name):
    dataset = gdal.Open(input_name)
    proj = dataset.GetProjection()
    trans = dataset.GetGeoTransform()
    img = dataset.ReadAsArray()

    # Resample
    res = (1000 / 60) * np.ones(img.shape)
    res[img < 3.0] = 4000 / 60                 # 4.0 km/h
    res[np.logical_and(img >=  3.0, img <  8.0)] = 3500 / 60  # 3.5 km/h
    res[np.logical_and(img >=  8.0, img < 15.0)] = 3000 / 60  # 3.0 km/h
    res[np.logical_and(img >= 15.0, img < 25.0)] = 2500 / 60  # 2.5 km/h
    res[np.logical_and(img >= 25.0, img < 35.0)] = 2000 / 60  # 2.0 km/h
    res[np.logical_and(img >= 35.0, img < 45.0)] = 1500 / 60  # 1.5 km/h
    res[img >= 45.0] = 1000 / 60               # 1.0 km/h

    # Write to GeoTiff
    nrows, ncols = res.shape
    driver = gdal.GetDriverByName("GTiff")
    out_raster = driver.Create(output_name, ncols, nrows, 1, gdal.GDT_Float64)
    out_raster.GetRasterBand(1).WriteArray(res)
    out_raster.SetGeoTransform(trans)
    out_raster.SetProjection(proj)
    out_raster.FlushCache()
    del out_raster

    compress(output_name, output_name)


if __name__ == "__main__":
    print("Hello World!")

    df = open_excel(fname, sheet_name)
    names, slopes, wspeeds = df["Name"], df["DEM-Slope"], df["Walk-Speed"]
    for (name, slope, wspeed) in zip(names, slopes, wspeeds):
        print("Current Region: %s" % name)
        slope_resample(slope, wspeed)

    print("Finished.")
