import neuroml as nml
from pyneuroml import pynml
from pyneuroml.lems import LEMSSimulation
import lems.api as lems
import hashlib



import numpy as np
from scipy.spatial import distance
import math

import network_utils as nu
import validate_network as valnet

def create_GoC_network( duration, dt, seed, N_goc=0, run=False, prob_type='Boltzmann', GJw_type='Vervaeke2010' ):


	goc_filename = 'GoC.cell.nml'
	goc_file = pynml.read_neuroml2_file( goc_filename )
	goc_type = goc_file.cells[0]
	
	GJ_filename = 'GapJuncCML.nml'
	GJ_file = pynml.read_neuroml2_file( GJ_filename )
	GJ_type = GJ_file.gap_junctions[0]

	MFSyn_filename = 'MF_GoC_Syn.nml'
	mfsyn_file = pynml.read_neuroml2_file( MFSyn_filename )
	MFSyn_type = mfsyn_file.exp_three_synapses[0]
	
	MF20Syn_filename = 'MF_GoC_SynMult.nml'
	mf20syn_file = pynml.read_neuroml2_file( MF20Syn_filename )
	MF20Syn_type = mf20syn_file.exp_three_synapses[0]
	
	# Distribute cells in 3D
	if N_goc>0:
		GoC_pos = nu.GoC_locate(N_goc)
	else:
		GoC_pos = nu.GoC_density_locate()
		N_goc = GoC_pos.shape[0]
		
	# get GJ connectivity
	GJ_pairs, GJWt = nu.GJ_conn( GoC_pos, prob_type, GJw_type )
	tmp1, tmp2 = valnet.gapJuncAnalysis( GJ_pairs, GJWt )
	print("Number of gap junctions per cell: ", tmp1)
	print("Net GJ conductance per cell:", tmp2)
	
	# Create pop List
	goc_pop = nml.Population( id=goc_type.id+"Pop", component = goc_type.id, type="populationList", size=N_goc )
	
	# Create NML document for network specification
	net = nml.Network( id="gocNetwork", type="networkWithTemperature" , temperature="23 degC" )
	net_doc = nml.NeuroMLDocument( id=net.id )
	net_doc.networks.append( net )
	net_doc.includes.append( goc_type )
	
	net.populations.append( goc_pop )
	
	#Add locations for GoC instances in the population:
	for goc in range(N_goc):
		inst = nml.Instance( id=goc )
		goc_pop.instances.append( inst )
		inst.location = nml.Location( x=GoC_pos[goc,0], y=GoC_pos[goc,1], z=GoC_pos[goc,2] )
		
	# Define input spiketrains
	input_type = 'spikeGenerator'#'spikeGeneratorPoisson'
	lems_inst_doc = lems.Model()
	mf_inputs = lems.Component( "MF_Input", input_type)
	mf_inputs.set_parameter("period", "2000 ms" )
	#mf_inputs.set_parameter("averageRate", "50 Hz")
	lems_inst_doc.add( mf_inputs )
	
	#synapse_type = 'alphaCurrentSynapse'
	#alpha_syn = lems.Component( "AlphaSyn", synapse_type)
	#alpha_syn.set_parameter("tau", "30 ms" )
	#alpha_syn.set_parameter("ibase", "200 pA")
	#lems_inst_doc.add( alpha_syn )
	
	# Define MF input population
	
	N_mf = 15
	#MF_pop = nml.Population(id=mf_inputs.id+"_pop", component=mf_inputs.id, type="populationList", size=N_mf)
	#net.populations.append( MF_pop )

	mf_type2 = 'spikeGeneratorPoisson'
	#mf_poisson = lems.Component( "MF_Poisson", mf_type2)
	#mf_poisson.set_parameter("averageRate", "5 Hz")
	#lems_inst_doc.add( mf_poisson )
	# adding in neuroml document instead of mf_poisson
	mf_poisson = nml.SpikeGeneratorPoisson( id = "MF_Poisson", average_rate="5 Hz" )
	net_doc.spike_generator_poissons.append( mf_poisson )
	
	net_doc.includes.append( goc_type )
	MF_Poisson_pop = nml.Population(id=mf_poisson.id+"_pop", component=mf_poisson.id, type="populationList", size=N_mf)
	net.populations.append( MF_Poisson_pop )
	MF_pos = nu.GoC_locate( N_mf )
	for mf in range( N_mf ):
		inst = nml.Instance(id=mf)
		MF_Poisson_pop.instances.append( inst )
		inst.location = nml.Location( x=MF_pos[mf,0], y=MF_pos[mf,1], z=MF_pos[mf,2] )
		
	# Setup Mf->GoC synapses
	#MFprojection = nml.Projection(id="MFtoGoC", presynaptic_population=MF_pop.id, postsynaptic_population=goc_pop.id, synapse=alpha_syn.id)
	#net.projections.append(MFprojection)

	MF2projection = nml.Projection(id="MF2toGoC", presynaptic_population=MF_Poisson_pop.id, postsynaptic_population=goc_pop.id, synapse=MFSyn_type.id)#alpha_syn.id
	net.projections.append(MF2projection)


	#Get list of MF->GoC synapse
	mf_synlist = nu.randdist_MF_syn( N_mf, N_goc, pConn=0.3)
	nMFSyn = mf_synlist.shape[1]
	for syn in range( nMFSyn ):
		mf, goc = mf_synlist[:, syn]
		conn2 = nml.Connection(id=syn, pre_cell_id='../{}/{}/{}'.format(MF_Poisson_pop.id, mf, mf_poisson.id), post_cell_id='../{}/{}/{}'.format(goc_pop.id, goc, goc_type.id), post_segment_id='0', post_fraction_along="0.5")
		MF2projection.connections.append(conn2)
		
		
	# Burst of MF input (as explicit input)
	mf_bursttype = 'transientPoissonFiringSynapse'
	mf_burst = lems.Component( "MF_Burst", mf_bursttype)
	mf_burst.set_parameter( "averageRate", "100 Hz" )
	mf_burst.set_parameter( "delay", "2000 ms" )
	mf_burst.set_parameter( "duration", "500 ms" )
	mf_burst.set_parameter( "synapse", MF20Syn_type.id )
	mf_burst.set_parameter( "spikeTarget", './{}'.format(MF20Syn_type.id) )
	lems_inst_doc.add( mf_burst )
	
	
	# Add few burst inputs
	n_bursts = 4
	gocPerm = np.random.permutation( N_goc )
	ctr = 0
	for gg in range(4):
		goc = gocPerm[gg]
		for jj in range( n_bursts ):
			inst = nml.ExplicitInput( id=ctr, target='../{}/{}/{}'.format(goc_pop.id, goc, goc_type.id), input=mf_burst.id, synapse=MF20Syn_type.id, spikeTarget='./{}'.format(MF20Syn_type.id))
			net.explicit_inputs.append( inst )
			ctr += 1
		
	
	'''
	one-to-one pairing of MF and GoC -> no shared inputs
	for goc in range(N_mf):
		#inst = nml.Instance(id=goc)
		#MF_pop.instances.append( inst )
		#inst.location = nml.Location( x=GoC_pos[goc,0], y=GoC_pos[goc,1], z=GoC_pos[goc,2]+100 )
		#conn = nml.Connection(id=goc, pre_cell_id='../{}/{}/{}'.format(MF_pop.id, goc, mf_inputs.id), post_cell_id='../{}/{}/{}'.format(goc_pop.id, goc, goc_type.id), post_segment_id='0', post_fraction_along="0.5")
		#MFprojection.connections.append(conn)

		goc2 = N_goc-goc-1
		inst2 = nml.Instance(id=goc)
		MF_Poisson_pop.instances.append( inst2 )
		inst2.location = nml.Location( x=GoC_pos[goc2,0], y=GoC_pos[goc2,1], z=GoC_pos[goc2,2]+100 )
		conn2 = nml.Connection(id=goc, pre_cell_id='../{}/{}/{}'.format(MF_Poisson_pop.id, goc, mf_poisson.id), post_cell_id='../{}/{}/{}'.format(goc_pop.id, goc2, goc_type.id), post_segment_id='0', post_fraction_along="0.5")
		MF2projection.connections.append(conn2)

	'''
	
	# Add electrical synapses
	GoCCoupling = nml.ElectricalProjection( id="gocGJ", presynaptic_population=goc_pop.id, postsynaptic_population=goc_pop.id )
	
	#print(GJ_pairs)
	gj = nml.GapJunction( id="GJ_0", conductance="426pS" )
	net_doc.gap_junctions.append(gj)
	nGJ = GJ_pairs.shape[0]
	for jj in range( nGJ ):
		#gj.append( lems.Component( "GJ_%d"%jj, 'gapJunction') )
		#gj[jj].set_parameter( "conductance", "%fnS"%(GJWt[jj]) )
		#gj = nml.GapJunction(id="GJ_%d"%jj, conductance="%fnS"%(GJWt[jj]))
		#net_doc.gap_junctions.append(gj)
		#lems_inst_doc.add( gj[jj] )
		#print("%fnS"%(GJWt[jj]*0.426))
		conn = nml.ElectricalConnectionInstanceW( id=jj, pre_cell='../{}/{}/{}'.format(goc_pop.id, GJ_pairs[jj,0], goc_type.id), pre_segment='1', pre_fraction_along='0.5', post_cell='../{}/{}/{}'.format(goc_pop.id, GJ_pairs[jj,1], goc_type.id), post_segment='1', post_fraction_along='0.5', synapse=gj.id, weight=GJWt[jj] )#synapse="GapJuncCML" synapse=gj.id , conductance="100E-9mS"
		# ------------ need to create GJ component
		GoCCoupling.electrical_connection_instance_ws.append( conn )
	
	net.electrical_projections.append( GoCCoupling )	
		
		
		
	net_filename = 'gocNetwork.nml'
	pynml.write_neuroml2_file( net_doc, net_filename )
	lems_filename = 'instances.xml'
	pynml.write_lems_file( lems_inst_doc, lems_filename, validate=False )

	simid = 'sim_gocnet'+goc_type.id
	ls = LEMSSimulation( simid, duration=duration, dt=dt, simulation_seed=seed )
	ls.assign_simulation_target( net.id )
	
	#ls.include_lems_file( 'Synapses.xml', include_included=False)
	#ls.include_lems_file( 'Inputs.xml', include_included=False)
	ls.include_neuroml2_file( net_filename)
	ls.include_neuroml2_file( goc_filename)
	ls.include_neuroml2_file( GJ_filename)
	ls.include_neuroml2_file( MFSyn_filename)
	ls.include_neuroml2_file( MF20Syn_filename)
	ls.include_lems_file( lems_filename, include_included=False)
	
	
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
	#res = pynml.run_lems_with_jneuroml_neuron( lems_simfile, max_memory="2G", only_generate_scripts = True, compile_mods = False, nogui=True, plot=False)
	res = pynml.run_lems_with_jneuroml_neuron( lems_simfile, max_memory="2G", compile_mods = False,nogui=True, plot=False)
	#res=True
	return res






	
if __name__ =='__main__':
	res = create_GoC_network( duration = 5000, dt=0.025, seed = 123, GJw_type = 'Szo16_oneGJ')
	print(res)