'''
Neuron simulator export for:

Components:
    null (Type: notes)
    GolgiNa (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiNaR (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiNaP (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiKA (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiSK2 (Type: ionChannelKS:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiKM (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiKV (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiBK (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiCaHVA (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiCaLVA (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiHCN1f (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiHCN1s (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiHCN2f (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiHCN2s (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    LeakConductance (Type: ionChannelPassive:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    Golgi_CALC (Type: decayingPoolConcentrationModel:  restingConc=5.0E-5 (SI concentration) decayConstant=7.69231E-4 (SI time) shellThickness=2.0378E-7 (SI length) Faraday=96485.3 (SI charge_per_mole) AREA_SCALE=1.0 (SI area) LENGTH_SCALE=1.0 (SI length))
    null (Type: notes)
    Golgi_CALC2 (Type: decayingPoolConcentrationModel:  restingConc=5.0E-5 (SI concentration) decayConstant=7.69231E-4 (SI time) shellThickness=2.0378E-7 (SI length) Faraday=96485.3 (SI charge_per_mole) AREA_SCALE=1.0 (SI area) LENGTH_SCALE=1.0 (SI length))
    GoCl (Type: cell)
    net1 (Type: networkWithTemperature:  temperature=296.15 (SI temperature))
    sim_goc1 (Type: Simulation:  length=0.15 (SI time) step=2.5E-5 (SI time))


    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.5.4
         org.neuroml.model   v1.5.4
         jLEMS               v0.9.9.1

'''

import neuron

import time
import sys

import hashlib
h = neuron.h
h.load_file("stdlib.hoc")

h.load_file("stdgui.hoc")

h("objref p")
h("p = new PythonObject()")

class NeuronSimulation():

    def __init__(self, tstop, dt, seed=12345):

        print("\n    Starting simulation in NEURON of %sms generated from NeuroML2 model...\n"%tstop)

        self.seed = seed
        self.randoms = []
        self.next_global_id = 0  # Used in Random123 classes for elements using random(), etc. 

        self.next_spiking_input_id = 0  # Used in Random123 classes for elements using random(), etc. 

        '''
        Adding simulation Component(id=sim_goc1 type=Simulation) of network/component: net1 (Type: networkWithTemperature:  temperature=296.15 (SI temperature))
        
        '''

        # Temperature used for network: 296.15 K
        h.celsius = 296.15 - 273.15

        # ######################   Population: gocpop
        print("Population gocpop contains 1 instance(s) of component: GoCl of type: cell")

        print("Setting the default initial concentrations for ca (used in GoCl) to 5.0E-5 mM (internal), 2.0 mM (external)")
        h("cai0_ca_ion = 5.0E-5")
        h("cao0_ca_ion = 2.0")

        print("Setting the default initial concentrations for ca2 (used in GoCl) to 5.0E-5 mM (internal), 2.0 mM (external)")
        h("ca2i0_ca2_ion = 5.0E-5")
        h("ca2o0_ca2_ion = 2.0")

        h.load_file("GoCl.hoc")
        a_gocpop = []
        h("{ n_gocpop = 1 }")
        h("objectvar a_gocpop[n_gocpop]")
        for i in range(int(h.n_gocpop)):
            h("a_gocpop[%i] = new GoCl()"%i)
            h("access a_gocpop[%i].Soma"%i)

            self.next_global_id+=1


        h("proc initialiseV_gocpop() { for i = 0, n_gocpop-1 { a_gocpop[i].set_initial_v() } }")
        h("objref fih_gocpop")
        h('{fih_gocpop = new FInitializeHandler(0, "initialiseV_gocpop()")}')

        h("proc initialiseIons_gocpop() { for i = 0, n_gocpop-1 { a_gocpop[i].set_initial_ion_properties() } }")
        h("objref fih_ion_gocpop")
        h('{fih_ion_gocpop = new FInitializeHandler(1, "initialiseIons_gocpop()")}')

        trec = h.Vector()
        trec.record(h._ref_t)

        h.tstop = tstop

        h.dt = dt

        h.steps_per_ms = 1/h.dt



        # ######################   File to save: sim_goc1.v.dat (Volts_file)
        # Column: gocpop[0]/v
        h(' objectvar v_v_Volts_file ')
        h(' { v_v_Volts_file = new Vector() } ')
        h(' { v_v_Volts_file.record(&a_gocpop[0].Soma.v(0.5)) } ')
        h.v_v_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)

        # ######################   File to save: sim_goc1.v.spikes (Events_file)
        h(' objectvar spiketimes_Events_file, t_spiketimes_Events_file ')
        h(' { spiketimes_Events_file = new Vector() } ')
        h(' { t_spiketimes_Events_file = new Vector() } ')
        h(' objref netConnSpike_Events_file, nil ')
        # Column: gocpop[0] (0) a_gocpop[0].Soma
        h(' a_gocpop[0].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 0) } ')

        # ######################   File to save: time.dat (time)
        # Column: time
        h(' objectvar v_time ')
        h(' { v_time = new Vector() } ')
        h(' { v_time.record(&t) } ')
        h.v_time.resize((h.tstop * h.steps_per_ms) + 1)

        self.initialized = False

        self.sim_end = -1 # will be overwritten

    def run(self):

        self.initialized = True
        sim_start = time.time()
        print("Running a simulation of %sms (dt = %sms; seed=%s)" % (h.tstop, h.dt, self.seed))

        h.run()

        self.sim_end = time.time()
        self.sim_time = self.sim_end - sim_start
        print("Finished NEURON simulation in %f seconds (%f mins)..."%(self.sim_time, self.sim_time/60.0))

        self.save_results()


    def advance(self):

        if not self.initialized:
            h.finitialize()
            self.initialized = True

        h.fadvance()


    ###############################################################################
    # Hash function to use in generation of random value
    # This is copied from NetPyNE: https://github.com/Neurosim-lab/netpyne/blob/master/netpyne/simFuncs.py
    ###############################################################################
    def _id32 (self,obj): 
        return int(hashlib.md5(obj).hexdigest()[0:8],16)  # convert 8 first chars of md5 hash in base 16 to int


    ###############################################################################
    # Initialize the stim randomizer
    # This is copied from NetPyNE: https://github.com/Neurosim-lab/netpyne/blob/master/netpyne/simFuncs.py
    ###############################################################################
    def _init_stim_randomizer(self,rand, stimType, gid, seed): 
        #print("INIT STIM  %s; %s; %s; %s"%(rand, stimType, gid, seed))
        rand.Random123(self._id32(stimType), gid, seed)


    def save_results(self):

        print("Saving results at t=%s..."%h.t)

        if self.sim_end < 0: self.sim_end = time.time()


        # ######################   File to save: time.dat (time)
        py_v_time = [ t/1000 for t in h.v_time.to_python() ]  # Convert to Python list for speed...

        f_time_f2 = open('time.dat', 'w')
        num_points = len(py_v_time)  # Simulation may have been stopped before tstop...

        for i in range(num_points):
            f_time_f2.write('%f'% py_v_time[i])  # Save in SI units...
        f_time_f2.close()
        print("Saved data to: time.dat")

        # ######################   File to save: sim_goc1.v.dat (Volts_file)
        py_v_v_Volts_file = [ float(x  / 1000.0) for x in h.v_v_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage

        f_Volts_file_f2 = open('sim_goc1.v.dat', 'w')
        num_points = len(py_v_time)  # Simulation may have been stopped before tstop...

        for i in range(num_points):
            f_Volts_file_f2.write('%e\t%e\t\n' % (py_v_time[i], py_v_v_Volts_file[i], ))
        f_Volts_file_f2.close()
        print("Saved data to: sim_goc1.v.dat")

        # ######################   File to save: sim_goc1.v.spikes (Events_file)

        f_Events_file_f2 = open('sim_goc1.v.spikes', 'w')
        h(' objref netConnSpike_Events_file ')
        spike_ids = h.spiketimes_Events_file.to_python()  
        spike_times = h.t_spiketimes_Events_file.to_python()
        for i, id in enumerate(spike_ids):
            # Saving in format: ID_TIME
            f_Events_file_f2.write("%i\t%s\n"%(id,spike_times[i]/1000.0))
        f_Events_file_f2.close()
        print("Saved data to: sim_goc1.v.spikes")

        save_end = time.time()
        save_time = save_end - self.sim_end
        print("Finished saving results in %f seconds"%(save_time))

        print("Done")

        quit()


if __name__ == '__main__':

    ns = NeuronSimulation(tstop=10000, dt=0.025, seed=12345)

    ns.run()

