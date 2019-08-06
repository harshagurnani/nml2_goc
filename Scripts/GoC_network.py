import neuroml as nml
from pyneuroml import pynml
from pyneuroml.lems import LEMSSimulation
import lems.api as lems


import numpy as np
from scipy.spatial import distance
import math

def create_GoC_network( N_goc=0, duration, dt, seed, run=False ):


	goc_filename = 'D:\Work\Comp Models\Learn Neuroml2\nml2_goc\Mechanisms\test_channel.cell.nml'
	goc_file = pynml.read_neuroml2_file( goc_file )
	goc_type = goc_file.component_types['simple_cell']
	
	
	# Distribute cells in 3D
	if N_goc>0:
		GoC_pos = GoC_locate(N_goc)
	else:
		GoC_pos = GoC_density_locate()
		N_goc = GoC.shape[0]
		
	# get GJ connectivity
	GJ_pairs, GJWt = GJ_conn( GoC_pos )
	
	
	# Create pop List
	goc_pop = nml.Population( id=goc_type.id+"Pop", component = goc_type.id, type="PopulationList", size=N_goc )
	
	# Create NML document for network specification
	net = nml.Network( id="gocNetwork" )
	net_doc = nml.NeuroMLDocument( id=net.id )
	net_doc.networks.append( net )
	net_doc.populations.append( goc_pop )
	
	
	#Add locations for GoC instances in the population:
	for goc in range(N_goc):
		inst = nml.Instance( id=goc )
		goc_pop.instances.append( inst )
		inst.location  nml.Location( x=GoC_pos[goc,0], y=GoC_pos[goc,1], z=GoC_pos[goc,2] )
		
		
	# Add electrical synapses
	GoCCoupling = nml.ElectricalProjection( id="gocGJ", presynaptic_population=goc_pop.id, postsynaptic_population=goc_pop.id )
	net.electrical_projections.append( GoCCoupling )
	
	for jj in range( len(GJ_pairs) ):
		conn = nml.ElectricalConnectionInstance( id=jj, preCell='../{}/{}/{}'.format(goc_pop.id, GJ_pairs[0,jj], goc_type.id), postCell='../{}/{}/{}'.format(goc_pop.id, GJ_pairs[1,jj], goc_type.id), preSegment="2", preFractionAlong=0, postSegment="2", postFractionAlong=0, synapse="GapJuncCML" )
		# ------------ need to create GJ component
		GCCoupling.electrical_connection_instances.append( conn )

	simid = 'sim_goc1'
	ls = LEMSSimulation( simid, duration=150, dt=0.025, target='net1' )
	
	#Load NeuroML components
	GoC_file_name = 'test_channel.cell.nml'#'Golgi.cell.nml'#'test_channel.cell.nml' #'simple_cell.cell.nml'	#Cell_Golgi.cell.nml
	ls.include_neuroml2_file( GoC_file_name )
	
	disp0 = 'dispaly0'
	ls.create_display(disp0, "Voltage", "-90", "50" )
	ls.add_line_to_display(disp0, "v", "gocpop[0]/v", "1mV", "#ffffff")

	of0 = 'Volts_file'
	ls.create_output_file(of0, "%s.v.dat"%simid)
	ls.add_column_to_output_file(of0, 'v', "gocpop[0]/v")
	
	#of1 = 'Na_file'
	#ls.create_output_file(of1, "%s.na.dat"%simid)
	#ls.add_column_to_output_file(of1, '0', "gocpop[0]/ina")
	
	eof0 = 'Events_file'
	ls.create_event_output_file(eof0, "%s.v.spikes"%simid,format='ID_TIME')
	ls.add_selection_to_event_output_file(eof0, '0', "gocpop[0]", "spike")

	#Create Lems file to run
	lems_simfile = ls.save_to_file()

	#res = pynml.run_lems_with_jneuroml_neuron( lems_simfile, max_memory="1G", compile_mods =False, nogui=True, plot=False)
	res = pynml.run_lems_with_jneuroml_neuron( lems_simfile, max_memory="1G", nogui=True, plot=False)
	#res=True
	return res


def GoC_locate( N=1, x=350,y=350,z=80):
	# Randomly distribute 'N' Golgi Cells in [0,x) X [0, y) X [0,z)
	GoC_pos = np.random.random( [N,3] ) * [x,y,z]  #in um
	return xx
	
def GoC_density_locate( density=4607, x=350,y=350,z=80):
	# Randomly distribute Golgi Cells in [0,x) X [0, y) X [0,z) based on density
	# Density in cells/mm3, dimensions [x,y,z] in um
	N = int( density * 1e-9*x*y*z  ) # units um->mm
	GoC_pos = np.random.random( [N,3] ) * [x,y,z]  #in um
	return xx

def GJ_conn( GoC_pos, prob_type='Boltzmann', GJw_type='Vervaeke2010' ):
	# modelling as single Gap junction between cells/mm3
	N_goc = GoC_pos.shape[0]
	radDist= distance.pdist( GoC_pos, 'euclidean' )
	
	if prob_type=='Boltzmann':
		isconn = connProb_Boltzmann( radDist )

	GJ_pairs = np.asarray(np.nonzero(isconn))
	GJ_pairs = [GJ_pairs[:,jj] for jj in range(GJ_pairs.shape[1]) if GJ_pairs[0,jj]<GJ_pairs[1,jj]]	#list of [cell1, cell2]
	
	# Gap junction conductance as a function of distance
	if GJw_type == 'Vervaeke2010':
		GJ_cond = set_GJ_strength_Vervaeke2010( [ radDist[GJ_pairs[jj][0], GJ_pairs[jj][1]] for jj in range(len(GJ_pairs)) ] ) #list of gj conductance for corresponding pair
	
	return GJ_pairs, GJ_cond

def connProb_Boltzmann( radDist )
	# Coupling prob as Boltzmann function - from Vervaeke 2010
	connProb = 1e-2 * (-1745 +  1836/( 1+np.exp((radDist-267)/39) ))
	connGen = np.random.random( radDist.shape )
	isconn = distance.squareform( (connProb - connGen)>0 ) # symmetric boolean matrix with diag=0 -> GJ or not
	return isconn
	
	
def set_GJ_strength_Vervaeke2010( radDist ):
	# Exponential fall-off of coupling coefficient, convert to GJ_cond
	CC = −2.3 + 29.7*np.exp(–radDist/70.4)	#Coupling coefficient
	GJw = 0.576 * np.exp(CC / 12.4) + 0.000590 * np.exp(CC / 2.79) – 0.564
	return GJw
	
if __name__ =='__main__':
	res = create_test_goc1()
	print(res)