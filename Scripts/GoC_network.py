import neuroml as nml
from pyneuroml import pynml
from pyneuroml.lems import LEMSSimulation
import lems.api as lems


import numpy as np
from scipy.spatial import distance
import math

def create_GoC_network( duration, dt, seed, N_goc=0, run=False ):


	goc_filename = 'simple_cell.cell.nml'
	goc_file = pynml.read_neuroml2_file( goc_filename )
	goc_type = goc_file.cells[0]
	
	GJ_filename = 'GapJuncCML.nml'
	GJ_file = pynml.read_neuroml2_file( GJ_filename )
	GJ_type = GJ_file.gap_junctions[0]
	
	# Distribute cells in 3D
	if N_goc>0:
		GoC_pos = GoC_locate(N_goc)
	else:
		GoC_pos = GoC_density_locate()
		N_goc = GoC.shape[0]
		
	# get GJ connectivity
	GJ_pairs, GJWt = GJ_conn( GoC_pos )
	
	
	# Create pop List
	goc_pop = nml.Population( id=goc_type.id+"Pop", component = goc_type.id, type="populationList", size=N_goc )
	
	# Create NML document for network specification
	net = nml.Network( id="gocNetwork" )
	net_doc = nml.NeuroMLDocument( id=net.id )
	net_doc.networks.append( net )
	#net_doc.cells.append( goc_type )
	net.populations.append( goc_pop )
	
	
	#Add locations for GoC instances in the population:
	for goc in range(N_goc):
		inst = nml.Instance( id=goc )
		goc_pop.instances.append( inst )
		inst.location = nml.Location( x=GoC_pos[goc,0], y=GoC_pos[goc,1], z=GoC_pos[goc,2] )
		
		
	# Add electrical synapses
	GoCCoupling = nml.ElectricalProjection( id="gocGJ", presynaptic_population=goc_pop.id, postsynaptic_population=goc_pop.id )
	net.electrical_projections.append( GoCCoupling )
	
	print(GJ_pairs)
	for jj in range( GJ_pairs.shape[0] ):
		cell1 = int(GJ_pairs[jj,0])
		print(cell1)
		cell2 = int(GJ_pairs[jj,1])
		print(cell2)
		conn = nml.ElectricalConnectionInstance( id=jj, pre_cell='../{}/{}/{}'.format(goc_pop.id, cell1, goc_type.id), pre_segment='1', pre_fraction_along='0.5', post_cell='../{}/{}/{}'.format(goc_pop.id, cell2, goc_type.id), post_segment='1', post_fraction_along='0.5', synapse="GapJuncCML" )
		# ------------ need to create GJ component
		GoCCoupling.electrical_connection_instances.append( conn )
		
		
	net_filename = 'gocNetwork.nml'
	pynml.write_neuroml2_file( net_doc, net_filename )
	

	simid = 'sim_gocnet'
	ls = LEMSSimulation( simid, duration=duration, dt=dt, simulation_seed=seed )
	ls.assign_simulation_target( net.id )
	
	ls.include_neuroml2_file( net_filename)
	ls.include_neuroml2_file( goc_filename)
	ls.include_neuroml2_file( GJ_filename)
	
	# Specify outputs
	
	eof0 = 'Events_file'
	ls.create_event_output_file(eof0, "%s.v.spikes"%simid,format='ID_TIME')
	for jj in range( goc_pop.size):
		ls.add_selection_to_event_output_file( eof0, jj, '{}/{}/{}'.format( goc_pop.id, jj, goc_type.id), 'spike' )
		
	of0 = 'Volts_file'
	ls.create_output_file(of0, "%s.v.dat"%simid)
	for jj in range( goc_pop.size ):
		ls.add_column_to_output_file(of0, jj, '{}/{}/{}/v'.format( goc_pop.id, jj, goc_type.id))
		
	#Create Lems file to run
	lems_simfile = ls.save_to_file()

	#res = pynml.run_lems_with_jneuroml( lems_simfile, max_memory="1G",nogui=True, plot=False)
	res = pynml.run_lems_with_jneuroml_neuron( lems_simfile, max_memory="2G",  nogui=True, plot=False)
	#res=True
	return res


def GoC_locate( N=1, x=350,y=350,z=80):
	# Randomly distribute 'N' Golgi Cells in [0,x) X [0, y) X [0,z)
	GoC_pos = np.random.random( [N,3] ) * [x,y,z]  #in um
	return GoC_pos
	
def GoC_density_locate( density=4607, x=350,y=350,z=80):
	# Randomly distribute Golgi Cells in [0,x) X [0, y) X [0,z) based on density
	# Density in cells/mm3, dimensions [x,y,z] in um
	N = int( density * 1e-9*x*y*z  ) # units um->mm
	GoC_pos = np.random.random( [N,3] ) * [x,y,z]  #in um
	return GoC_pos

def GJ_conn( GoC_pos, prob_type='Boltzmann', GJw_type='Vervaeke2010' ):
	# modelling as single Gap junction between cells/mm3
	N_goc = GoC_pos.shape[0]
	radDist= distance.pdist( GoC_pos, 'euclidean' )
	
	if prob_type=='Boltzmann':
		isconn = connProb_Boltzmann( radDist )

	GJ_pairs = np.asarray(np.nonzero(isconn))
	GJ_pairs = np.asarray([GJ_pairs[:,jj] for jj in range(GJ_pairs.shape[1]) if GJ_pairs[0,jj]<GJ_pairs[1,jj]])	# N_pairs x 2 array = [cell1, cell2] of each pair
	
	radDist = distance.squareform(radDist)
	
	# Gap junction conductance as a function of distance
	if GJw_type == 'Vervaeke2010':
		GJ_cond = set_GJ_strength_Vervaeke2010( np.asarray([ radDist[GJ_pairs[jj,0], GJ_pairs[jj,1]] for jj in range(GJ_pairs.shape[0]) ]) ) #list of gj conductance for corresponding pair
	return GJ_pairs, GJ_cond

def connProb_Boltzmann( radDist ):
	# Coupling prob as Boltzmann function - from Vervaeke 2010
	connProb = 1e-2 * (-1745 +  1836/( 1+np.exp((radDist-267)/39) ))
	connGen = np.random.random( radDist.shape )
	isconn = distance.squareform( (connProb - connGen)>0 ) # symmetric boolean matrix with diag=0 -> GJ or not
	return isconn
	
	
def set_GJ_strength_Vervaeke2010( radDist ):
	# Exponential fall-off of coupling coefficient, convert to GJ_cond
	CC = -2.3 + 29.7*np.exp(-radDist/70.4)	#Coupling coefficient
	GJw = 0.576 * np.exp(CC/12.4) + 0.00059 * np.exp(CC/2.79) - 0.564
	return GJw
	
if __name__ =='__main__':
	res = create_GoC_network( duration = 200, dt=0.025, seed = 12345, N_goc=10)
	print(res)