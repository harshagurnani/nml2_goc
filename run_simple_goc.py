import neuroml as nml
from pyneuroml import pynml
from pyneuroml.lems import LEMSSimulation
import lems.api as lems

def create_test_goc1():
	simid = 'sim_goc1'
	ls = LEMSSimulation( simid, duration=1500, dt=0.0025, target='net1' )
	
	#Load NeuroML components
	GoC_file_name = 'simple_cell.cell.nml'	#Cell_Golgi.cell.nml
	ls.include_neuroml2_file( GoC_file_name )
	
	disp0 = 'dispaly0'
	ls.create_display(disp0, "Voltage", "-90", "50" )
	ls.add_line_to_display(disp0, "v", "gocpop[0]/v", "1mV", "#ffffff")

	of0 = 'Volts_file'
	ls.create_output_file(of0, "%s.v.dat"%simid)
	ls.add_column_to_output_file(of0, 'v', "gocpop[0]/v")
	
	eof0 = 'Events_file'
	ls.create_event_output_file(eof0, "%s.v.spikes"%simid,format='ID_TIME')
	ls.add_selection_to_event_output_file(eof0, '0', "gocpop[0]", "spike")

	#Create Lems file to run
	lems_simfile = ls.save_to_file()

	res = pynml.run_lems_with_jneuroml( lems_simfile, max_memory="1G", nogui=True, plot=False)

	return res

if __name__ =='__main__':
	res = create_test_goc1()
	print(res)