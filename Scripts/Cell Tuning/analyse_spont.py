import pyelectro.analysis as analysis
import numpy as np
import matplotlib as pyplot

	
	
def get_spont( nFiles ):

	spikeFreq = np.zeros( nFiles )
	for fid in range( nFiles ): 
		fname = format(fid, '05d' )
		v = np.genfromtxt('sim_gocnet_GoC_{}.v.dat'.format(fname) )
		spikes = np.genfromtxt('sim_gocnet_GoC_{}.v.spikes'.format(fname) )
		if not np.isnan(v[-1,1]):
			spikeFreq[fid] = analysis.mean_spike_frequency(spikes[:,1]*1e3)
		else:
			spikeFreq[fid] = np.nan
			
	temp = spikeFreq
	temp[np.isnan(temp)]=0
	useParams = np.where( abs(temp-7) <=2 )
	
	print(useParams, spikeFreq[useParams] )
	
	return spikeFreq