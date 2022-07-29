# -*- coding: utf-8 -*-
"""
Created on Tue Jul 26 17:29:29 2022

@authors: Y. Reza and T. Wilkes
"""

import inspect
import os
import re

from matplotlib import pyplot as plt
import numpy as np
import pandas as pd


def movingaverage(array, n=None):
    # https://stackoverflow.com/questions/14313510/how-to-calculate-rolling-moving-average-using-python-numpy-scipy
    ret = np.cumsum(array, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


def listofmatches(list2parse, regex=None):
    if regex is None:
        regex = ".*"

    matches = [
        re.match(regex, item)[0]
        for item in list2parse
        if re.match(regex, item) is not None
        ]

    return matches


def data_parse():
    #   ==========
    #   Parse data
    #   ==========

    # Find available data to parse
    datadir = os.path.abspath(os.path.join(
            inspect.getfile(lambda: None), os.pardir
            ))
    datadir = os.path.join(
        datadir, "data"
        )

    root, directory, filenames = list(os.walk(datadir))[0]

    print("Available files:")
    for i, filename in enumerate(filenames):
        print(f"[{i:03}] {filename}")

    # Ask the user what file should be analysed
    while(True):
        try:
            filechoice = int(input(
                "Select a number representing the file you want >>> "
                ))
        except ValueError:
            pass
        else:
            if filechoice < len(filenames):
                break
            else:
                pass
        print("Invalid selection!")

    filename = filenames[filechoice]

    # Open the file and parse the data inside into a dataframe
    with open(f"{datadir}\\{filename}", "r") as f:
        firstlineitems = f.readline().split(",")

    # Firstly extract the names of the columns from the columnated data
    columnnames = [re.findall(r" ?(.*\[.*\])", x) for x in firstlineitems]
    columnnames = [x[0] if len(x) > 0 else None for x in columnnames]

    # Produce a dataframe with the appropriate column names
    dataframe = pd.read_csv(f"{datadir}\\{filename}", sep=",", header=None)
    dataframe.columns = columnnames
    del dataframe[None]
    columnnames.remove(None)

    # The columnated data is a mix of column name and data - extract data only
    for _, column in enumerate(columnnames):
        dataframe[column] = [
            np.float64(re.findall(r" ?.*\[.*\](.*)", x)[0])
            for x in dataframe[column][:]
            ]
    return dataframe


def data_plot(dataframe, xcolumnname="System Uptime[ms]"):
    #   ============================
    #   Automatically plot some data
    #   ============================

    columnnames = dataframe.columns.tolist()

    # Determine the x-axis to be plotted
    if xcolumnname is None or xcolumnname not in columnnames:
        print(f"{xcolumnname} is not a recognised column name, skipping arg")
        xcolumnname = "Datapoint [-]"
        xs = np.arange(len(dataframe))
    else:
        xs = dataframe[xcolumnname]

    def autoplotxy(title, regex):

        matches = listofmatches(list2parse=columnnames, regex=regex)
        if len(matches) > 0:
            fig, ax = plt.subplots(
                nrows=1, ncols=1, dpi=140, sharex=True, num=title
                )
            fig.suptitle(title)

            sources = [
                columnname.rsplit(sep=" ", maxsplit=1)[0]
                for columnname in matches
                ]
            measures = [
                columnname.rsplit(sep=" ", maxsplit=1)[1]
                for columnname in matches
                ]

            if len(set(measures)) == 1 and len(set(sources)) == len(matches):
                ylabels = measures
                labels = sources
            elif len(set(sources)) == 1 and len(set(measures)) == len(matches):
                ylabels = sources
                labels = measures
            else:
                ylabels = ["Value"] * len(sources)
                labels = [
                    f"{sources[i]} {measures[i]}"
                    for i in range(len(sources))
                    ]

            ylimits = [np.nan, np.nan]
            for i, columnname in enumerate(matches):
                source, measure = columnname.rsplit(sep=" ", maxsplit=1)
                ax.plot(
                    xs, dataframe[columnname],
                    label=labels[i]
                    )
                ylimits = [
                    np.nanmin([
                        ylimits[0],
                        np.nanpercentile(dataframe[columnname], 5) / 1.3
                        ]),
                    np.nanmax([
                        ylimits[1],
                        np.nanpercentile(dataframe[columnname], 95) * 1.3
                        ])
                    ]
            ax.set_xlabel(xcolumnname)
            ax.set_ylabel(ylabels[i])
            ax.set_ylim(*ylimits)
            ax.legend()
            ax.grid()

        return ax

    autoplotxy(title="Dynamic Pressure", regex=r".+ q\[Pa\]")
    autoplotxy(title="Sensor Temperature", regex=r".+ Tpackage\[C\]")
    autoplotxy(title="Battery Health", regex=r".+ VBATTcell[0-9]\[V\]")
    autoplotxy(title="Speeds", regex=r"(.+ speed\[m/s\])|(.+ VTAS\[m/s\])")
    return

    # Calculating GPS ground speed
    
    # a = 6378137
    # f = 1 / 298.257223563
    
    
    # def earth_radius(theta):
    #     # Calculates GPS earth radius from latitude
    
    #     top = a * (1 - f)
    #     bottom = np.sqrt(1 + (((np.cos(theta)) ** 2) * f * (f - 2)))
    
    #     return top / bottom
    
    
    # def array_delta(array):
    #     # Converts df to an array and subtracts [:-1] from [1:]
    
    #     array = np.asarray(array)
    #     delta_array = array[1:] - array[:-1]
    
    #     return delta_array
    
    
    # def array_midpoint(array):
    #     # Converts df to an array and averages [:-1] from [1:]
    
    #     array = np.asarray(array)
    #     delta_array = (array[1:] + array[:-1]) / 2
    
    #     return delta_array
    
    
    # delta_lat_rad = np.radians(array_delta(dataframe["GPS_0 latitude[deg]"]))
    # delta_lng_rad = np.radians(array_delta(dataframe["GPS_0 longitude[deg]"]))
    
    # lat_midpoints = np.radians(array_midpoint(dataframe["GPS_0 latitude[deg]"]))
    
    # earth_radii = earth_radius(lat_midpoints)
    
    
    # delta2_lat_rad = np.where(delta_lat_rad != 0, delta_lat_rad, np.nan)
    # delta2_lng_rad = np.where(delta_lat_rad != 0, delta_lng_rad, np.nan)
    # delta2_lat_rad = np.where(delta_lng_rad != 0, delta_lat_rad, np.nan)
    # delta2_lng_rad = np.where(delta_lng_rad != 0, delta_lng_rad, np.nan)
    
    # dist_lat = (earth_radii * delta_lat_rad)[~np.isnan(delta2_lat_rad)]
    # dist_lng = (earth_radii * delta_lng_rad)[~np.isnan(delta2_lat_rad)]
    # dist = np.sqrt(dist_lat ** 2 + dist_lng ** 2)
    
    # times = array_midpoint(dataframe["System Uptime[ms]"]) / 1000
    # times = times[~np.isnan(delta2_lat_rad)]
    # dtimes = array_delta(dataframe["System Uptime[ms]"]) / 1000
    # dtimes = dtimes[~np.isnan(delta2_lat_rad)]
    # speeds = dist/dtimes
    
    # delta2_lat_rad = delta2_lat_rad[~np.isnan(delta2_lat_rad)]
    # delta2_lng_rad = delta2_lng_rad[~np.isnan(delta2_lng_rad)]
    
    # print(f"speeds: {speeds}")
    # print(np.sum(dist))

    # plt.show()
    # fig, ax = plt.subplots(dpi=140)
    # ax.plot(times, speeds)
    
    
    #print(dataframe["GPS_0 latitude[deg]"][0], end=",")
    #print(dataframe["GPS_0 longitude[deg]"][0])
    #print(np.asarray(dataframe["GPS_0 latitude[deg]"])[-1], end=",")
    #print(np.asarray(dataframe["GPS_0 longitude[deg]"])[-1])

if __name__ == "__main__":

    dataframe = data_parse()
    data_plot(dataframe=dataframe)

    # Plot the airspeed and altitude against time
    fig, ax1 = plt.subplots(1, 1, figsize=(11, 4), dpi=140)
    fig.suptitle("Airspeed and Altitude vs. Time")
    ax1.set_ylim(0, 20)
    ax2 = ax1.twinx()

    ax1.plot(
        dataframe["System Uptime[ms]"],
        dataframe["Airdata VTAS[m/s]"],
        c='blue', label="VTAS"
        )
    ax1.plot(
        dataframe["System Uptime[ms]"],
        dataframe["GPS_0 speed[m/s]"],
        c='orange', label="GPS"
        )
    ax2.plot(
        dataframe["System Uptime[ms]"],
        dataframe["VL53L1X_0 s[m]"],
        c='magenta', label="LIDAR"
        )

    ax1.set_xlabel("System Uptime[ms]")
    ax1.set_ylabel("Speed [m/s]")
    ax1.grid()
    ax2.set_ylabel("Alt [m]")

    fig.legend(loc=8, ncol=3)
    fig.subplots_adjust(bottom=0.225)

    # Plot the GPS data
    fig, ax = plt.subplots(dpi=140)

    im = ax.scatter(
        dataframe["GPS_0 longitude[deg]"],
        dataframe["GPS_0 latitude[deg]"],
        c=dataframe["VL53L1X_0 s[m]"], cmap="viridis", zorder=3
        )
    ax.scatter(
        dataframe["GPS_0 longitude[deg]"][0],
        dataframe["GPS_0 latitude[deg]"][0],
        marker='o', s=100, c="r", zorder=3, label="Start"
        )
    ax.scatter(
        np.asarray(dataframe["GPS_0 longitude[deg]"])[-1],
        np.asarray(dataframe["GPS_0 latitude[deg]"])[-1],
        marker='X', s=100, c="r", zorder=3, label="Finish"
        )

    ax.ticklabel_format(useOffset=False, style='plain')
    ax.set_aspect('equal')
    plt.xticks(rotation=45)
    ax.grid(zorder=0)

    ax.legend()
    fig.colorbar(im, ax=ax, label="Altitude [m]")
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    fig.suptitle("Position coloured by Altitude")
