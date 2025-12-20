import numpy as np
import matplotlib.pyplot as plt

# Dataset Synthesis
satellites = {'garmin':{'cov':[[20,15],[15,30]],'bias':[2,-3],'size':60},
              'lowrance':{'cov':[[40,0],[0,50]],'bias':[-3,0],'size':80},
              'flir':{'cov':[[5,-10],[-10,60]],'bias':[1,-1],'size':50}}
location = [-15,30]

for name,s in satellites.items():
    mean = np.array(s['bias'])+np.array(location)
    cov = s['cov']
    size = s['size']
    s['data'] = np.random.multivariate_normal(mean,cov,size)
    plt.plot(s['data'][:,0],s['data'][:,1],'o')
plt.show()

# Analysis
inv_cov = np.zeros((2,2))
mean = np.array([0,0])
for s in satellites:
    H_s = np.linalg.inv(satellites[s]['cov'])
    data_s = satellites[s]['data']
    inv_cov = inv_cov + len(data_s)*H_s
    mean = mean + H_s @ np.sum(data_s,axis=0)
mean = np.linalg.pinv(inv_cov)@ mean
P = np.linalg.pinv(inv_cov)

print(mean)
print(P)