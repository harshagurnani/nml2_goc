# Generates file params_file.pkl, which enumerates parameter sets to be run in parallel
# Run as: python initialize_network_params.py

import numpy as np
import pickle as pkl



import sys
sys.path.append('../')
import network_utils as nu

def get_simulation_params(simid,
                          nGoC=0,
                          volume=[350,350,80],
						  GoC_loc_type='Density',
						  GoC_density=4607,
						  GJ_dist_type='Boltzmann',
						  GJ_wt_type='Szo16_oneGJ',
						  protocol='iSteps',
						  testGoC=[0],
						  minI=0,
						  maxI=200,
						  iStep=10,
						  iDur=400,
						  iRest=500,
						 ):

	params={}
	params["nGoC"], params["GoC_pos"] = nu.locate_GoC( nGoC, volume, GoC_loc_type, GoC_density, seed=simid)
	
	params["GJ_pairs"], params["GJ_wt"], params["GJ_loc"] = nu.GJ_conn( params["GoC_pos"], GJ_dist_type, GJ_wt_type, nDend=3, seed=simid )

	params["Test_GoC"]=testGoC
	params["iAmp"] = np.arange(minI, maxI+iStep/2, iStep)
	params["nSteps"]= params["iAmp"].shape[0]
	params["iDuration"]=iDur
	params["iRest"]=iRest
	
	return params
	
	
    
if __name__ =='__main__':

	nSim = 10
	params_list = [ get_simulation_params(simid) for simid in range(nSim) ]
	
	file = open('params_file.pkl','wb')
	pkl.dump(params_list,file); file.close()