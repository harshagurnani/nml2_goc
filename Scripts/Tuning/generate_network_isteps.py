import neuroml as nml
from pyneuroml import pynml
from pyneuroml.lems import LEMSSimulation
import lems.api as lems
import hashlib

import numpy as np
from scipy.spatial import distance
import math
import pickle as pkl
import sys
from pathlib import Path
import network_tuning_params as inp

def create_GoC_network( duration, dt, seed, runid, run=False):

	### ---------- Load Params
	noPar = True
	pfile = Path('params_file.pkl')
	if pfile.exists():
		print('Reading parameters from file:')
		file = open('params_file.pkl','rb')
		params_list = pkl.load(file)
		if len(params_list)>runid:
			p = params_list[runid]
			file.close()
	if noPar:
		p = inp.get_simulation_params( runid )
    
	### ---------- Component types
	goc_filename = 'GoC.cell.nml'							# Golgi cell with channels
	goc_file = pynml.read_neuroml2_file( goc_filename )
	goc_type = goc_file.cells[0]
	goc_ref = nml.IncludeType( href=goc_filename )

	gj = nml.GapJunction( id="GJ_0", conductance="426pS" )	# GoC synapse
	
	### --------- Populations

	# Build network to specify cells and connectivity
	net = nml.Network( id="gocNetwork", type="networkWithTemperature" , temperature="23 degC" )
		
	# Create GoC population
	goc_pop = nml.Population( id=goc_type.id+"Pop", component = goc_type.id, type="populationList", size=p["nGoC"] )
	for goc in range( p["nGoC"] ):
		inst = nml.Instance( id=goc )
		goc_pop.instances.append( inst )
		inst.location = nml.Location( x=p["GoC_pos"][goc,0], y=p["GoC_pos"][goc,1], z=p["GoC_pos"][goc,2] )
	net.populations.append( goc_pop )
	
	# Create NML document for network specification
	net_doc = nml.NeuroMLDocument( id=net.id )
	net_doc.networks.append( net )
	net_doc.includes.append( goc_ref )
	net_doc.gap_junctions.append(gj)
	
	
	### ------------ Connectivity

	### 1. Input Current to one cell
	ctr=0
	for goc in p["Test_GoC"]:
		for jj in range( p["nSteps"] ):
			input_id='stim_{}'.format(ctr)
			istep = nml.PulseGenerator(id=input_id,delay='{} ms'.format(p["iDuration"]*jj+p["iRest"]*(jj+1)),duration='{} ms'.format(p["iDuration"]),amplitude='{} pA'.format(p["iAmp"][jj]))
			net_doc.pulse_generators.append(istep)
			
			input_list = nml.InputList( id='ilist_{}'.format(ctr), component=istep.id, populations=goc_pop.id )
			curr_inj = nml.Input('0', target="../%s[%i]"%(goc_pop.id, goc), destination="synapses")
			input_list.input.append(curr_inj)
			net.input_lists.append(input_list)
			ctr+=1

	### 2. Electrical coupling between GoCs
		
	GoCCoupling = nml.ElectricalProjection( id="gocGJ", presynaptic_population=goc_pop.id, postsynaptic_population=goc_pop.id )
	net.electrical_projections.append( GoCCoupling )
	dend_id = [1,2,5]
	for jj in range( p["GJ_pairs"].shape[0] ):
		conn = nml.ElectricalConnectionInstanceW( id=jj, pre_cell='../{}/{}/{}'.format(goc_pop.id, p["GJ_pairs"][jj,0], goc_type.id), pre_segment=dend_id[p["GJ_loc"][jj,0]], pre_fraction_along='0.5', post_cell='../{}/{}/{}'.format(goc_pop.id, p["GJ_pairs"][jj,1], goc_type.id), post_segment=dend_id[p["GJ_loc"][jj,1]], post_fraction_along='0.5', synapse=gj.id, weight=p["GJ_wt"][jj] )
		GoCCoupling.electrical_connection_instance_ws.append( conn )
		
		
	### --------------  Write files
		
	net_filename = 'gocNetwork.nml'
	pynml.write_neuroml2_file( net_doc, net_filename )

	simid = 'sim_gocnet_'+goc_type.id+'_run_{}'.format(runid)
	ls = LEMSSimulation( simid, duration=duration, dt=dt, simulation_seed=seed )
	ls.assign_simulation_target( net.id )
	ls.include_neuroml2_file( net_filename)
	ls.include_neuroml2_file( goc_filename)

	
	# Specify outputs
	eof0 = 'Events_file'
	ls.create_event_output_file(eof0, "%s.v.spikes"%simid,format='ID_TIME')
	for jj in range( goc_pop.size):
		ls.add_selection_to_event_output_file( eof0, jj, '{}/{}/{}'.format( goc_pop.id, jj, goc_type.id), 'spike' )
		
	of0 = 'Volts_file'
	ls.create_output_file(of0, "%s.v.dat"%simid)
	ctr=0
	for jj in p["Test_GoC"]:
		ls.add_column_to_output_file(of0, jj, '{}/{}/{}/v'.format( goc_pop.id, ctr, goc_type.id))
		ctr+=1
		
	#Create Lems file to run
	lems_simfile = ls.save_to_file()

	if run:
		res = pynml.run_lems_with_jneuroml_neuron( lems_simfile, max_memory="2G", nogui=True, plot=False)
	else:
		res = pynml.run_lems_with_jneuroml_neuron( lems_simfile, max_memory="2G", only_generate_scripts = True, compile_mods = False, nogui=True, plot=False)
	
	
	return res

	
if __name__ =='__main__':
	runid=0
	if len(sys.argv)>1:
		runid=int(sys.argv[1])
	print('Generating network using parameters for runid=', runid)
	res = create_GoC_network( duration = 5000, dt=0.025, seed = 123, runid=runid)
	print(res)