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
import initialize_cell_params as icp

def create_GoC( runid ):

	### ---------- Load Params
	noPar = True
	pfile = Path('cellparams_file.pkl')
	if pfile.exists():
		print('Reading parameters from file:')
		file = open('cellparams_file.pkl','rb')
		params_list = pkl.load(file)
		if len(params_list)>runid:
			p = params_list[runid]
			file.close()
	if noPar:
		p = icp.get_channel_params( runid )
    
	


	# Creating document for cell
	gocID = 'GoC_{}'.format(runid)
	goc = nml.Cell( id=gocID )		#--------simid	
	cell_doc = nml.NeuroMLDocument( id=gocID )
	cell_doc.cells.append( goc )
	
	
	### Load morphology
	morpho_fname='GoC.cell.nml'
	morpho_file = pynml.read_neuroml2_file( morpho_fname )
	morpho = morpho_file.cells[0].morphology
	cell_doc.includes.append( nml.IncludeType( href=morpho_fname ) )
	goc.morphology = morpho
	
	### ---------- Channels
	na_fname 	= 'Golgi_Na.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=na_fname) )
	nar_fname 	= 'Golgi_NaR.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=nar_fname) )
	nap_fname = 'Golgi_NaP.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=nap_fname) )
	
	ka_fname 	= 'Golgi_KA.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=ka_fname) )
	sk2_fname 	= 'Golgi_SK2.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=sk2_fname) )
	km_fname 	= 'Golgi_KM.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=km_fname) )
	kv_fname 	= 'Golgi_KV.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=kv_fname) )
	bk_fname	= 'Golgi_BK.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=bk_fname) )
	
	cahva_fname	= 'Golgi_CaHVA.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=cahva_fname) )
	calva_fname = 'Golgi_CaLVA.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=calva_fname) )
	
	hcn1f_fname	= 'Golgi_HCN1f.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=hcn1f_fname) )
	hcn1s_fname	= 'Golgi_HCN1s.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=hcn1s_fname) )
	hcn2f_fname	= 'Golgi_HCN2f.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=hcn2f_fname) )
	hcn2s_fname	= 'Golgi_HCN2s.channel.nml'
	cell_doc.includes.append( nml.IncludeType( href=hcn2s_fname) )
	
	#leak_fname	= 'Golgi_lkg.channel.nml'
	#leak_ref 	= nml.IncludeType( href=leak_fname)
	#cell_doc.includes.append( nml.IncludeType( href=leak_fname) )
	calc_fname	= 'Golgi_CALC.nml'
	cell_doc.includes.append( nml.IncludeType( href=calc_fname) )
	calc 		= pynml.read_neuroml2_file( calc_fname).decaying_pool_concentration_models[0]
	
	### ------Biophysical Properties
	biophys = nml.BiophysicalProperties()
	goc.biophysical_properties = biophys
	
	# Intracellular properties
	res = nml.Resistivity( p["ra"] )		# --------- "0.1 kohm_cm" 
	ca_species = nml.Species( id="ca", ion="ca", concentration_model=calc.id, initial_concentration ="5e-5 mM", initial_ext_concentration="2 mM" )
	intracellular = nml.IntracellularProperties(  )
	intracellular.resistivities.append( res )
	intracellular.species.append( ca_species )
	biophys.intracellular_properties = intracellular 
	
	# Membrane properties ------- cond
	memb = nml.MembraneProperties()
	biophys.membrane_properties = memb 
	
	'''
	chan_leak = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(leak_fname).ion_channel[0].id,
									cond_density=p["leak_cond"],
									erev="-55 mV",
									ion="non_specific",
									id="Leak"
								  )
	memb.channel_densities.append( chan_leak)	
	'''
	chan_na   = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(na_fname).ion_channel[0].id,
									cond_density=p["na_cond"],
									erev="87.39 mV",
									ion="na",
									id="Golgi_Na_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_na)							  
	chan_nap  = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(nap_fname).ion_channel[0].id,
									cond_density=p["nap_cond"],
									erev="87.39 mV",
									ion="na",
									id="Golgi_NaP_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_nap)
	chan_nar  = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(nar_fname).ion_channel[0].id,
									cond_density=p["nar_cond"],
									erev="87.39 mV",
									ion="na",
									id="Golgi_NaR_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_nar)
	chan_ka   = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(ka_fname).ion_channel[0].id,
									cond_density=p["ka_cond"],
									erev="-84.69 mV",
									ion="k",
									id="Golgi_KA_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_ka)
	chan_sk   = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(sk2_fname).ion_channel_kses[0].id,
									cond_density=p["sk2_cond"],
									erev="-84.69 mV",
									ion="k",
									id="Golgi_KAHP_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_sk)
	chan_kv   = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(kv_fname).ion_channel[0].id,
									cond_density=p["kv_cond"],
									erev="-84.69 mV",
									ion="k",
									id="Golgi_KV_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_kv)
	chan_km   = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(km_fname).ion_channel[0].id,
									cond_density=p["km_cond"],
									erev="-84.69 mV",
									ion="k",
									id="Golgi_KM_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_km)
	chan_bk   = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(bk_fname).ion_channel[0].id,
									cond_density=p["bk_cond"],
									erev="-84.69 mV",
									ion="k",
									id="Golgi_BK_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_bk)
	chan_h1f  = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(hcn1f_fname).ion_channel[0].id,
									cond_density=p["hcn1f_cond"],
									erev="-20 mV",
									ion="h",
									id="Golgi_hcn1f_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_h1f)
	chan_h1s  = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(hcn1s_fname).ion_channel[0].id,
									cond_density=p["hcn1s_cond"],
									erev="-20 mV",
									ion="h",
									id="Golgi_hcn1s_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_h1s)
	chan_h2f  = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(hcn2f_fname).ion_channel[0].id,
									cond_density=p["hcn2f_cond"],
									erev="-20 mV",
									ion="h",
									id="Golgi_hcn2f_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_h2f)
	chan_h2s  = nml.ChannelDensity( ion_channel=pynml.read_neuroml2_file(hcn2s_fname).ion_channel[0].id,
									cond_density=p["hcn2s_cond"],
									erev="-20 mV",
									ion="h",
									id="Golgi_hcn2s_soma_group",
									segment_groups="soma_group"
								  )
	memb.channel_densities.append( chan_h2s)
	chan_hva  = nml.ChannelDensityNernst( 	ion_channel=pynml.read_neuroml2_file(cahva_fname).ion_channel[0].id,
											cond_density=p["cahva_cond"],
											ion="ca",
											id="Golgi_Ca_HVA_soma_group",
											segment_groups="soma_group"
										)
	memb.channel_density_nernsts.append( chan_hva)
	chan_lva  = nml.ChannelDensityNernst( 	ion_channel=pynml.read_neuroml2_file(calva_fname).ion_channel[0].id,
											cond_density=p["calva_cond"],
											ion="ca",
											id="Golgi_Ca_LVA_soma_group",
											segment_groups="soma_group"
										)
	memb.channel_density_nernsts.append( chan_lva)
	
	memb.spike_threshes.append(nml.SpikeThresh("0 mV"))
	memb.specific_capacitances.append(nml.SpecificCapacitance("1.0 uF_per_cm2"))
	memb.init_memb_potentials.append( nml.InitMembPotential("-60 mV") )
	
		
	goc_filename = '{}.cell.nml'.format(gocID)
	pynml.write_neuroml2_file( cell_doc, goc_filename )

	
	
	return True

	
if __name__ =='__main__':
	runid=0
	if len(sys.argv)>1:
		runid=int(sys.argv[1])
	print('Generating Golgi cell using parameters for simid=', runid)
	res = create_GoC( runid=runid)
	print(res)