import rospy
import sys
import time
import datetime
import math
import os
import numpy as np
from os.path import isfile, join


def LastNlines(fname, N):
    # assert statement check
    # a condition
    assert N >= 0

    # declaring variable
    # to implement
    # exponential search
    pos = N + 1

    # list to store
    # last N lines
    lines = []

    # opening file using with() method
    # so that file get closed
    # after completing work
    with open(fname) as f:

        # loop which runs
        # until size of list
        # becomes equal to N
        while len(lines) <= N:

            # try block
            try:
                # moving cursor from
                # left side to
                # pos line from end
                f.seek(-pos, 2)

                # exception block
            # to hadle any run
            # time error
            except IOError:
                f.seek(0)
                break

            # finally block
            # to add lines
            # to list after
            # each iteration
            finally:
                lines = list(f)

                # increasing value
            # of variable
            # exponentially
            pos *= 2

    # returning the
    # whole list
    # which stores last
    # N lines
    return lines[-N:]


if __name__ == '__main__':
    # Map dimensions and cell size
    xmin = -45  # meters
    xmax = 55  # meters
    ymin = -35  # meters
    ymax = 30;  # meters
    cell_size = 0.1  # meters

    # starting and ending time, time interval.
    time_interval = 1800  # seconds
    t_ini = 0  # seconds from 1970
    t_end = 0  # seconds from 1970

    cwd = os.getcwd() + '/data'
    onlyfiles = sorted([f for f in os.listdir(cwd) if isfile(join(cwd, f))])
    print(onlyfiles)
    #for i, _ in enumerate(onlyfiles):
     #   onlyfiles[i] =  onlyfiles[i]

    for filename in onlyfiles:

        # DAY 1 - CALCULATE HISTOGRAMS AND LOAD THEM INTO FREMEN
        input_file_name = './data/' +filename
        #print(input_file_name)
        #raw_input("Press Enter to continue...")
        output_file_name = './datah/' +filename[:-4] + '-histograms.txt'
        #print(output_file_name)
        #raw_input("Press Enter to continue...")

        with open('data/' +filename ) as f:
            #print(filename[2:])
            #raw_input("Press Enter to continue...")
            first_line = f.readline()
            k = first_line.split(',')
            #print(first_line )
            #raw_input("Press Enter to continue...")
            print(filename)
            t_ini = int(float(k[0]))
        
        #print(t_ini)
        #raw_input("Press Enter to continue...")
        k = LastNlines('data/' +filename, 1)[0].split(',')
        t_end = int(float(k[0]))

        #print(t_end)
        #raw_input("Press Enter to continue...")
        print("Creating flowmap histograms...")
        os.system("python ./scripts/create_flowmap_histograms.py " + input_file_name + " " + output_file_name + " " + str(
            xmin) + " " + str(xmax) + " " + str(ymin) + " " + str(ymax) + " " + str(cell_size) + " " + str(
            t_ini) + " " + str(t_end) + " " + str(time_interval))
        print("Done")

        print("Loading histograms to FreMEn...")
        os.system("python ./scripts/load_histograms.py " + output_file_name)
        print("Done")


    # PREDICT ORIENTATIONS
    predictions_file_name = "./datao/predicted_20121107_order2sssss.txt"
    times_to_predict = './times/times_to_predict_20121107_10min_0to24.txt'
    order = 2
    print("Making predictions...")
    os.system(
       'python ./scripts/get_predictions.py ' + str(order) + ' ' + times_to_predict + ' ' + predictions_file_name)
    print("Done")

    # PLOT FLOWMAP
    #predictions_file_name = "./data/predicted_20121107_order1" + str(human) + ".txt"
    rows = int((ymax - ymin) / cell_size)
    cols = int((xmax - xmin) / cell_size)
    print("Plotting flowmap...")
    os.system('python ./scripts/plot_flowmap.py ' + predictions_file_name + ' ' + str(rows) + ' ' + str(cols))
    print("Done - Images in folder predicted_flow_img ")

   # os.system('python ./scripts/delete_fremen.py ')



