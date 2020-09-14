#!/usr/bin/env python

# import rospy
import sys
import time
# from std_msgs.msg import String
# from fremenarray.msg import FremenArrayActionGoal, FremenArrayActionResult
import numpy as np
import math


def result_callback(data):
    print
    data.result.message


if __name__ == '__main__':

    if len(sys.argv) == 11:
        input_file_name =  sys.argv[1]
        output_file_name =  sys.argv[2]
        # Map dimensions and cell size
        xmin = int(sys.argv[3])  # meters
        xmax = int(sys.argv[4])  # meters
        ymin = int(sys.argv[5])  # meters
        ymax = int(sys.argv[6])  # meters
        cell_size = float(sys.argv[7])  # meters

        # starting and ending time, time interval.
        t_ini = int(sys.argv[8])  # seconds from 1970 1351038020  # i
        t_end = int(sys.argv[9])  # seconds from 1970 1351078043  # 
        time_interval = int(sys.argv[10])  ##seconds

    else:
        print
        "This scripts need 10 parameters: input_file_name, output_file_name,xmin,xmax,ymin,ymax,cell_size,t_ini,t_end,time_interval "
        sys.exit(1)

    # Read the file
    ifile = open(input_file_name, "r")
    ifile.close()
    ifile = open(input_file_name, "r")
    ofile = open(output_file_name, "a")

    # input_data = np.genfromtxt(input_file_name, delimiter=',')

    rows = int((ymax - ymin) / cell_size)
    cols = int((xmax - xmin) / cell_size)

    # start the histogram building
    # last_ite = 0;
    bin_count_matrix = np.zeros((rows, cols))
    last_time_frame = t_ini
    time_frame_difference = 0
    # print('------------------------')
    # print(t_ini)
    # raw_input("Press Enter to continue...")
    input_data = ifile.readline()
    input_data = input_data.split(",")
    print('------------------------')
    flag = False
    disk = []
    scanner = 10
    for i in range(-scanner,scanner+1):
        for j in range(-scanner, scanner+1):
            if i**2+j**2<=scanner**2:
                disk.append([i,j])
    checker = np.zeros((rows, cols))
    for t_now in range(t_ini, t_end, time_interval):

        # check people moving to create the histogram
        # for i in range(last_ite, len(input_data)):
        while True:
            inpData = float(input_data[0])
            #print(inpData)
            '''
            print(input_data[0])
            raw_input("Press Enter to continue...")
            print(last_ite)
            raw_input("Press Enter to continue...")
            print(t_now)
            raw_input("Press Enter to continue...")
            print(time_interval)
            raw_input("Press Enter to continue...")
            print(input_data[1])
            raw_input("Press Enter to continue...")
            '''
            if (inpData > (t_now + time_interval - 0.0001)):
                # print('breaker')
                # raw_input("Press Enter to continue...")
                break
            if inpData != last_time_frame:
                # print('breakerNie')
                # raw_input("Press Enter to continue...")
                time_frame_difference = inpData - last_time_frame
                last_time_frame = inpData
                checker = np.zeros((rows, cols))
            if (inpData >= t_now and inpData < (t_now + time_interval - 0.0001)):
                row = int(math.floor((-1 / (ymax - (ymax - cell_size))) * float(input_data[2]) + (
                        1 - (-1 / (ymax - (ymax - cell_size))) * ymax)))
                col = int(math.floor((1 / ((xmin + cell_size) - xmin)) * float(input_data[1]) + (
                        1 - (1 / ((xmin + cell_size) - xmin)) * xmin)))
                # print(input_data[0])
                # raw_input("Press Enter to continue...")
                # print(col)
                # raw_input("Press Enter to continue...")
                for d in disk:
                    if checker[d[0]+row,col+d[1]]==0:
                        checker[d[0]+row,col+d[1]]=1
                        bin_count_matrix[d[0]+row,col+d[1]] = bin_count_matrix[d[0]+row,col+d[1]]  + time_frame_difference*100;
            input_data = ifile.readline()
            if input_data == '':
                Flag = True
                break
            input_data = input_data.split(",")

        # normalize matrix

        for r in range(0, rows):
            for c in range(0, cols):
                bin_count_matrix[r, c] = bin_count_matrix[r, c] / time_interval;

        # Save  the matrix in the output file
        ofile.write(str(t_now))
        ofile.write(",")
        bin_count_matrix_1d = np.reshape(bin_count_matrix, (1, rows * cols))
        for i in range(0, rows * cols):
            ofile.write(str(float(bin_count_matrix_1d[0][i])))
            if i != rows * cols - 1:
                ofile.write(",")
        ofile.write("\n")

        # Restart matrix
        bin_count_matrix = np.zeros((rows, cols))

        # Verbosity
        print(time.ctime(int(t_now)))

        if flag:
            break

    ifile.close()
    ofile.close()

