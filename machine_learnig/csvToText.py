import os
from os.path import isfile, join
import csv


cwd = os.getcwd()+'/data'
onlyfiles = [f for f in os.listdir(cwd) if isfile(join(cwd, f))]
for i,_ in enumerate(onlyfiles):
    onlyfiles[i] = 'data/'+onlyfiles[i]
for file in onlyfiles:
  if file[-1]=='v':
    with open(file) as csvfile:
        dir = file[:-4]+'.txt'
        file = open(dir, "w")
        spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        i=0
        for row in spamreader:
            k=row[0].split(',')
            stringer = k[0]+','+str(float(k[2])/1000)+','+str(float(k[3])/1000)+','+k[6]+','+k[1]+'\n'
            file.write(stringer)
            i=i+5
    file.close()
