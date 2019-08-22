import numpy as np
from scipy.spatial import distance
import math
from matplotlib import pyplot

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
	
	pp=pyplot.figure(1,figsize=(8,4)) # Default figsize is (8,6)
	for jj in range(nCells):
		pyplot.plot( allV[:,0], allV[:,jj+1]+jj*0.1, figure=pp )
	pyplot.xlabel('time (s)', figure=pp)
	pyplot.ylabel('mV', figure=pp)	
	pp.savefig( 'Traces_{}.png'.format(ID))


def spike_raster( spikes_file, ID='', marker='.' ):
	allSpikes = np.loadtxt( spikes_file )
	pp=pyplot.figure(1,figsize=(8,4)) # Default figsize is (8,6)
	pyplot.scatter( allSpikes[:,1], allSpikes[:,0], figure=pp )
	pyplot.xlabel('time (s)', figure=pp)
	pyplot.ylabel('Cell #', figure=pp)	
	pp.savefig( 'spikeRaster_{}.png'.format(ID))


def get_plots_for_sim( simid, traces=True, spikes=True ):

	if traces:
		plot_traces( '{}.v.dat'.format(simid), simid )
		
	if spikes:
		spike_raster( '{}.v.spikes'.format(simid), simid )