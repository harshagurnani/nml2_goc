import numpy as np
from scipy.spatial import distance
import math
from matplotlib import pyplot
import pickle as pkl
'''
Check ntwork connectivity measures
'''

def gapJuncAnalysis( GJ_pairs, GJ_wt, nGoC=0 ):
	if nGoC==0:
		nGoC = np.amax( GJ_pairs)
	
	nGJ_per_cell = np.zeros(nGoC)
	net_GJ = np.zeros( nGoC )
	for goc in range( nGoC ):
		nGJ_per_cell[ goc ]= (GJ_pairs == goc).sum()
		net_GJ[ goc ] = GJ_wt[ GJ_pairs[:,0] == goc ].sum() + GJ_wt[ GJ_pairs[:,1] == goc ].sum()
		
	return nGJ_per_cell, net_GJ
	
	
def get_spike_freq( spikes, nCells ):
	if isinstance(spikes, str):
		spikes = np.loadtxt( spikes )	#interpret str as filename and load file
	return [1/np.mean(np.diff(spikes[spikes[:,0]==jj,1])) for jj in range(nCells)]
	
	
	
def plot_traces( vfile, ID ):
	allV = np.loadtxt( vfile )
	nCells = allV.shape[1]-1
	
	pp=pyplot.figure(1,figsize=(8,6)) # Default figsize is (8,6)
	for jj in range(nCells):
		pyplot.plot( allV[:,0], allV[:,jj+1]+jj*0.1, figure=pp )
	pyplot.xlabel('time (s)', figure=pp)
	pyplot.ylabel('mV', figure=pp)	
	pp.savefig( 'Traces_{}.png'.format(ID))
	pyplot.show()
	pyplot.pause(1)


def spike_raster( spikes_file, ID='', marker='.' ):
	allSpikes = np.loadtxt( spikes_file )
	pp=pyplot.figure(1,figsize=(8,6)) # Default figsize is (8,6)
	pyplot.scatter( allSpikes[:,1], allSpikes[:,0], figure=pp )
	pyplot.xlabel('time (s)', figure=pp)
	pyplot.ylabel('Cell #', figure=pp)	
	pp.savefig( 'spikeRaster_{}.png'.format(ID))


def get_plots_for_sim( simid, traces=True, spikes=True ):

	if traces:
		plot_traces( '{}.v.dat'.format(simid), simid )
		
	if spikes:
		spike_raster( '{}.v.spikes'.format(simid), simid )
		
		
def get_FI_curve( runid, simid ):
	file = open('params_file.pkl','rb')
	params_list = pkl.load(file)
	p = params_list[runid]
	file.close()
	
	
	spikes = np.loadtxt( '{}.v.spikes'.format(simid) )
	nTest = len( p["Test_GoC"] )
	inj_i = p["iAmp"] #in pA
	print( nTest )
	fi = np.zeros( [inj_i.shape[0], nTest] )
	for goc in range(nTest):
		resp=spikes[spikes[:,0]==p["Test_GoC"][goc],1]
		print(resp)
		for cn in range( inj_i.shape[0] ):
			t1 = (p["iDuration"]*cn+p["iRest"]*(cn+1))*1e-3
			t2 = t1+p["iDuration"]*1e-3
			fi[cn,goc] = sum([1 for x in resp if (x>=t1 and x<=t2)])/p["iDuration"]*1000 # Hz
	return fi
	