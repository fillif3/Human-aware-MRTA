import os
import numpy as np
from scipy.optimize import minimize,basinhopping
import math
import operator

from random import seed
from random import random
from random import randrange


def getFiles(dir):
    files = os.listdir(dir)
    return files

def loadFromFile(dir):
    f = open(dir, "r")
    text =f.readline().split(',')
    f.close()
    t = float(text[0])
    input=[]
    for i in range(int((len(text)-1)/2)):
        input.append([float(text[2*i+1]),float(text[2*i+2])])
    return t,input

# Create a random subsample from the dataset with replacement
def subsample(dataset, ratio=1.0):
    sample = list()
    n_sample = round(len(dataset) * ratio)
    while len(sample) < n_sample:
        index = randrange(len(dataset))
        sample.append(dataset[index])
    return sample


def evaluationZering(w,xss,times,zering = 10):
    global errors
    errors=[]
    #error = 0
    i=0
    for t,xs in zip(times,xss):
        i=i+1
        output=0
        for x in xs:
            o = func(x,w)
            output +=o
        #error = error + abs(t-output)
        errors.append(abs(t-output))
    for _ in range(zering):
        index, value = max(enumerate(errors), key=operator.itemgetter(1))
        errors[index]=0
    error = sum(errors)
    return error/i

def evaluation(w,xss,times):
    global errors
    errors=[]
    error = 0
    i=0
    for t,xs in zip(times,xss):
        i=i+1
        output=0
        for x in xs:
            o = func(x,w)
            output +=o
        error = error + abs(t-output)
        errors.append(abs(t-output))
    return error/i



def funcfdsfsd(x,w): #done
    d = x[0]
    p = x[1]/100
    return d*(p**3*w[3]+p**2*w[2]+p*w[1]+w[0])

def funcxxxx(x,w): #done
    d = x[0]
    p = x[1]/100
    return d*(p**2*w[2]+p*w[1]+w[0])

def func534(x,w): #done
    d = x[0]
    p = x[1]/100
    return d*(p**5*w[5]+p**4*w[4]+p**3*w[3]+p**2*w[2]+p*w[1]+w[0])

def funcbnbn(x,w):
    d = x[0]
    p = x[1]/100
    return d*(p**7*w[7]+p**6*w[6]+p**5*w[5]+p**4*w[4]+p**3*w[3]+p**2*w[2]+p*w[1]+w[0])

def funcbcvbc(x,w):
    d = x[0]
    p = x[1]/100
    return d*(p**9*w[9]+p**8*w[8]+p**7*w[7]+p**6*w[6]+p**5*w[5]+p**4*w[4]+p**3*w[3]+p**2*w[2]+p*w[1]+w[0])

def func2233(x,w):
    d = x[0]
    p = x[1]/100
    return d*(p**11*w[11]+p**10*w[10]+p**9*w[9]+p**8*w[8]+p**7*w[7]+p**6*w[6]+p**5*w[5]+p**4*w[4]+p**3*w[3]+p**2*w[2]+p*w[1]+w[0])

def funcnb(x,w): #powtórzyć
    d = x[0]
    p = x[1]/100
    if (w[1]-p*w[2])==0:
        div = 0.0000001
    else:
        div = (w[1]-p*w[2])
    return d*w[0]/div

def funcdsadsa(x,w): #powtórzyć
    d = x[0]
    p = x[1]/100
    if (w[1]-p*w[2])==0:
        div = 0.0000001
    else:
        div = (w[1]-p*w[2])
    return d*(w[0]+w[3]*p)/div

def funcdsa(x,w):  #done
    d = x[0]
    p = x[1]/100
    return d*(math.exp(p*w[0]))

def funcsada(x,w): #done
    d = x[0]
    p = x[1]/100
    return d*(w[1]*math.exp(p*w[0]))

def funcqew(x,w):#pow
    d = x[0]
    p = x[1]/100
    try:
        ans = d * (w[1] * math.exp(p * w[0])+w[2])
    except OverflowError:
        ans = float('inf')
    return ans

def func(x,w):#pow
    d = x[0]
    p = x[1]/100
    valu=0
    for wbg in w:
        valu = valu + wbg[2]+wbg[1] * ((math.exp(p * wbg[0])))


    return d*valu/20


def funcvx(x,w):
    d = x[0]
    p = x[1]/100
    try:
        ans = d * (w[1] * math.exp(p * w[0])+w[3]*p + w[2])
    except OverflowError:
        ans = float('inf')
    return ans


cwd = os.getcwd()
dir = cwd + '/Roads'
dir2 = cwd + '/Roads2'
files = getFiles(dir)
files2 = getFiles(dir)

times = []
inputs = []
times2 = []
inputs2 = []
params =[([-4.17699287e+02, -1.08627863e-01,  1.09808276e+00]), ([-3.70180305e+02, -1.00329942e-01,  1.10137480e+00]), ([9.49323002, 0.0185826 , 0.99693687]), ([-3.77874570e+02, -8.51557857e-02,  1.09506640e+00]), ([-22.59693699,  -0.16517132,   1.16854772]), ([-3.87555003e+02, -1.24251176e-01,  1.11918610e+00]), ([-3.59178448e+02, -1.12191192e-01,  1.10007722e+00]), ([-25.08546915,  -0.15708927,   1.15806156]), ([-3.48934540e+02, -9.27217896e-02,  1.10692705e+00]), ([-3.47478777e+02, -1.05950442e-01,  1.10584813e+00]), ([7.45566368, 0.11658583, 0.87143745]), ([-3.63769677e+02, -9.49591321e-02,  1.10260995e+00]), ([-4.09240921e+02, -1.18647285e-01,  1.11401421e+00]), ([-3.42554953e+02, -1.09232705e-01,  1.10305384e+00]), ([-3.63846775e+02, -1.12728766e-01,  1.11590260e+00]), ([-3.81657284e+02, -1.26810246e-01,  1.10983712e+00]), ([-3.22674476e+02, -7.46118932e-02,  1.08528139e+00]), ([-10.02003386,  -0.24592477,   1.26129726]), ([-3.50964195e+02, -8.17886231e-02,  1.09144538e+00]), ([-7.28339891e+03, -1.44416974e-01,  1.12389937e+00])]
#[[9.34671181, 0.02124643, 1.01423483],[10.07051751,  0.01074498,  1.02326757], [-3.92630803e+02, -9.41345188e-02,  1.09658120e+00], [-3.98385465e+02, -1.35265214e-01,  1.11541105e+00], [-3.68749357e+02, -1.46972910e-01,  1.12440325e+00], [-5.04453448, -0.73707706,  1.71646605], [-4.23369483e+02, -1.11269897e-01,  1.11351551e+00], [-3.93994507e+02, -8.66099303e-02,  1.09087052e+00], [-3.92377044e+02, -9.96985381e-02,  1.10057868e+00],[-3.68159500e+02, -1.50675295e-01,  1.12608097e+00], [-2.99079717e+02, -1.26842950e-01,  1.11373091e+00], [-3.38814714, -0.7100144 ,  1.70436143], [-3.84321638e+02, -1.38899445e-01,  1.11184917e+00], [-9.1451457 , -0.24955107,  1.26137016], [-2.43765638, -2.09141232,  3.01925375], [8.28768559, 0.03852672, 0.98675687],[-18.83105603,  -0.11357018,   1.12954339],[9.82254885, 0.01091328, 1.02868596], [8.41613187, 0.03017208, 0.99373187], [-3.50823297e+02, -9.43723955e-02,  1.09699327e+00]]

    #[1,1,1] #[-0.07716514,  0.3810509,  1.03891526,  0.00304528]
timesComputed=[]
for file in files:
    time, input = loadFromFile(dir+'/'+file)
    times.append(time)
    inputs.append(input)
for file in files2:
    time, input = loadFromFile(dir+'/'+file)
    times2.append(time)
    inputs2.append(input)
timeComputed = evaluation(params,inputs2,times2)
bagTimes=[]
bagInputs=[]
sorter = list(range(len(inputs)))
for _ in range(20):
    bTime=[]
    bInput=[]
    bSorter = subsample(sorter)
    for sor in bSorter:
        bTime.append(times[sor])
        bInput.append(inputs[sor])
    bagTimes.append(bTime)
    bagInputs.append(bInput)
    #timesComputed.append(timeComputed)
print(timeComputed)
outs = []
d=0
for i,t in zip(bagInputs,bagTimes):
    d=d+1
    print(d)
    minimizer_kwargs = {"args":(i,t), "tol":1e-3, "method":"Powell"}
#res = minimize(evaluation, [params, ], args=(inputs, times), tol=1e-3, method="Powell")
    ret = basinhopping(evaluation, [params, ], minimizer_kwargs=minimizer_kwargs,niter=20,disp=True,niter_success=10)
    outs.append(ret.x)
try:
    #print (errors)
    pass
except:
    pass

print(outs)

