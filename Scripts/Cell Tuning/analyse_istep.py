import pyelectro.analysis as analysis
import numpy as np
import matplotlib as pyplot
import pickle as pkl

import sys
sys.path.append('../')
import validate_network as vn


def analyse_exp( fid, minI=-75, maxI=200, iStep=25, iDur=400, iRest=500 ):
	fname = format(fid, '05d' )
	v = np.genfromtxt('sim_gocnet_istep_GoC_{}.v.dat'.format(fname) )
	spikes = np.genfromtxt('sim_gocnet_istep_GoC_{}.v.spikes'.format(fname) )	
	iAmp = np.arange(minI, maxI+iStep/2, iStep)
	nSteps = iAmp.shape[0]
	dt = 25*1e-3  #ms

	passIstep = False
	# 1. Fi curve
	FI = np.zeros(nSteps)
	Adapt = np.zeros(nSteps)
	for jj in range(nSteps):
		interval = [iRest + jj*(iRest+iDur), (jj+1)*(iRest+iDur) ]	# ms
		currSpikes = spikes[(spikes[:,1]>=interval[0]*1e-3) & (spikes[:,1]<=interval[1]*1e-3),1]
		FI[jj] = (currSpikes.shape[0]) *1e3/iDur
		# 2. Adaptation
		if currSpikes.shape[0]>2:
			isi = np.diff(currSpikes)
			Adapt[jj] = isi[-1]/isi[0]
		else:
			Adapt[jj] = np.nan
	
	if not np.isnan(v[-1,1]):
	
		slope = analysis.linear_fit( iAmp[FI>0], FI[FI>0])
		
		# 3.Sag at I = -75 pa// jj=1
		interval = [int(iRest/dt), int((iRest+iDur)/dt) ]	# ms
		last50ms = int(50/dt)
		sagAmp = np.mean(v[interval[1]-last50ms:interval[1],1]) - np.min(v[interval[0]:interval[1],1])
		passIstep = True
	else:
		slope = np.nan
		sagAmp = np.nan
		
	results = {'FI': FI, 'slope': slope, 'sagAmp': sagAmp, 'adaptation' : Adapt, 'pass': passIstep}
	return results
	
def get_istep_results( nFiles ):
	file = open('useParams_SpontFreq_7_pm_2.pkl','rb')
	p = pkl.load(file)["useParams"][0]
	file.close()
	
	allRes = [ analyse_exp(fid) for fid in p[range(nFiles)] ]
	useParams = [ p[fid] for fid in range(nFiles) if allRes[fid]["pass"] ]
	return allRes, useParams