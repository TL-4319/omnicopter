import numpy as np
from scipy import interpolate
from numpy import linalg as LA

##########################################################################
#inputs for a function
traj_file = open('/home/hume-users/btheresa/src/PX4-Autopilot/launch/omni_traj/scripts/halo_aimdown_50Hz_accel')
transit = 5 #transfer time
p0 = np.array([0, 0 , 0.75])
q0 = np.array([1,0,0,0])
########################################################################
#read in data and take the first several data points to get the trajectory
M = np.loadtxt(traj_file,delimiter=",")
Mf = M.astype(float)
pos = Mf[:20,:3]

#initial known times
dt = 1/50
tread = np.arange(0,(len(pos))*dt,dt)
tstart = tread+transit
tstart = np.insert(tstart,0,0)

#initial known positions
xstart = pos[:,0]
xstart = np.insert(xstart,0,p0[0])
ystart = pos[:,1]
ystart = np.insert(ystart,0,p0[1])
zstart = pos[:,2]
zstart = np.insert(zstart,0,p0[2])


#compute new positions for transfer trajectory
tfind = np.arange(0,transit,dt)

xf = interpolate.interp1d(tstart,xstart,'cubic')
xnew = xf(tfind)
yf = interpolate.interp1d(tstart,ystart,'cubic')
ynew = yf(tfind)
zf = interpolate.interp1d(tstart,zstart,'cubic')
znew = zf(tfind)

#compute transfer trajectory velcoities and accelerations
vx = (xnew[1:]-xnew[0:-1])/dt
vy = (ynew[1:]-ynew[0:-1])/dt
vz = (znew[1:]-znew[0:-1])/dt

ax = (vx[1:]-vx[0:-1])/dt
ay = (vy[1:]-vy[0:-1])/dt
az = (vz[1:]-vz[0:-1])/dt

ax = np.insert(ax,0,[0, 0])
ay = np.insert(ay,0,[0, 0])
az = np.insert(az,0,[0, 0])

#attitude
att = Mf[:20,6:10]

#initial quaternions
qwstart = att[:,0]
qwstart = np.insert(qwstart,0,q0[0])
qxstart = att[:,1]
qxstart = np.insert(qxstart,0,q0[1])
qystart = att[:,2]
qystart = np.insert(qystart,0,q0[2])
qzstart = att[:,3]
qzstart = np.insert(qzstart,0,q0[3])

#transfer trajectory quaternions
qwf = interpolate.interp1d(tstart,qwstart,'cubic')
qwnew = qwf(tfind)
qxf = interpolate.interp1d(tstart,qxstart,'cubic')
qxnew = qxf(tfind)
qyf = interpolate.interp1d(tstart,qystart,'cubic')
qynew = qyf(tfind)
qzf = interpolate.interp1d(tstart,qzstart,'cubic')
qznew = qzf(tfind)

#normalize quaternion
qarr = np.array([qwnew,qxnew,qynew,qznew])
qnorm = LA.norm(qarr,axis=0)
qarr = qarr/qnorm

#compute angular velocity
qdot = (qarr[:,1:]-qarr[:,0:-1])/dt
qw = qarr[0,:]
qx = qarr[1,:]
qy = qarr[2,:]
qz = qarr[3,:]

warr = np.zeros((3,len(qw)))
ind = np.arange(0,len(qw)-1,1)
for i in ind:
	W = np.array([[-qx[i], qw[i], -qz[i], qy[i]], [-qy[i], qz[i], qw[i], -qx[i]], [-qz[i], -qy[i], qx[i], qw[i]]])
	warr[:,i+1] = 2*W @ qdot[:,i]

#################################################################
#return xnew ynew znew ax ay az  qarr.tranpose warr.transpose
