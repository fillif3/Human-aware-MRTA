import os
import sys

if len(sys.argv)==1:
    a=1360714200#-7*24*60*60
else:
    a=float(sys.argv[1])
ifile = open('times_to_predict_20121107_10min_0to24.txt', "w")
for _ in range(144):
   ifile.write(str(a)+"\n")
   a=a+600

  




