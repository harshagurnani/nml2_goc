'''
Neuron simulator export for:

Components:
    gocNetwork (Type: networkWithTemperature:  temperature=296.15 (SI temperature))
    null (Type: notes)
    LeakConductance (Type: ionChannelPassive:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiNa (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiKV (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    null (Type: notes)
    GolgiNaP (Type: ionChannelHH:  conductance=1.0E-11 (SI conductance))
    simple_cell (Type: cell)
    net1 (Type: networkWithTemperature:  temperature=296.15 (SI temperature))
    null (Type: notes)
    GapJuncCML (Type: gapJunction:  conductance=1.0000000000000002E-12 (SI conductance))
    sim_gocnet (Type: Simulation:  length=0.2 (SI time) step=2.5E-5 (SI time))


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
        Adding simulation Component(id=sim_gocnet type=Simulation) of network/component: gocNetwork (Type: networkWithTemperature:  temperature=296.15 (SI temperature))
        
        '''

        # Temperature used for network: 296.15 K
        h.celsius = 296.15 - 273.15

        # ######################   Population: simple_cellPop
        print("Population simple_cellPop contains 10 instance(s) of component: simple_cell of type: cell")

        h.load_file("simple_cell.hoc")
        a_simple_cellPop = []
        h("{ n_simple_cellPop = 10 }")
        h("objectvar a_simple_cellPop[n_simple_cellPop]")
        for i in range(int(h.n_simple_cellPop)):
            h("a_simple_cellPop[%i] = new simple_cell()"%i)
            h("access a_simple_cellPop[%i].Soma"%i)

            self.next_global_id+=1

        h("{ a_simple_cellPop[0].position(158.05548, 109.10128, 76.53857) }")
        h("{ a_simple_cellPop[1].position(30.594244, 143.80536, 57.756615) }")
        h("{ a_simple_cellPop[2].position(65.61163, 5.291004, 9.313237) }")
        h("{ a_simple_cellPop[3].position(249.94395, 241.57002, 78.05853) }")
        h("{ a_simple_cellPop[4].position(3.7272198, 93.293915, 22.83718) }")
        h("{ a_simple_cellPop[5].position(311.8748, 75.57683, 10.074388) }")
        h("{ a_simple_cellPop[6].position(288.1411, 54.85699, 75.083755) }")
        h("{ a_simple_cellPop[7].position(112.928314, 231.76147, 23.973436) }")
        h("{ a_simple_cellPop[8].position(41.179295, 69.801735, 74.458664) }")
        h("{ a_simple_cellPop[9].position(263.83517, 171.57458, 70.37245) }")

        h("proc initialiseV_simple_cellPop() { for i = 0, n_simple_cellPop-1 { a_simple_cellPop[i].set_initial_v() } }")
        h("objref fih_simple_cellPop")
        h('{fih_simple_cellPop = new FInitializeHandler(0, "initialiseV_simple_cellPop()")}')

        h("proc initialiseIons_simple_cellPop() { for i = 0, n_simple_cellPop-1 { a_simple_cellPop[i].set_initial_ion_properties() } }")
        h("objref fih_ion_simple_cellPop")
        h('{fih_ion_simple_cellPop = new FInitializeHandler(1, "initialiseIons_simple_cellPop()")}')

        # ######################   Electrical Projection: gocGJ
        print("Adding electrical projection: gocGJ from simple_cellPop to simple_cellPop, with 10 connection(s)")

        h("objectvar syn_gocGJ_GapJuncCML_A[10]")
        h("objectvar syn_gocGJ_GapJuncCML_B[10]")

        # Elect Connection 0: cell 0, seg 1 (0.5) [0.5 on a_simple_cellPop[0].Section_1] -> cell 1, seg 1 (0.5) [0.5 on a_simple_cellPop[1].Section_1], weight: 1.0
        h("a_simple_cellPop[0].Section_1 { syn_gocGJ_GapJuncCML_A[0] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[1].Section_1 { syn_gocGJ_GapJuncCML_B[0] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[0].vpeer, a_simple_cellPop[1].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[0].vpeer, a_simple_cellPop[0].Section_1.v(0.5)")

        # Elect Connection 1: cell 0, seg 1 (0.5) [0.5 on a_simple_cellPop[0].Section_1] -> cell 7, seg 1 (0.5) [0.5 on a_simple_cellPop[7].Section_1], weight: 1.0
        h("a_simple_cellPop[0].Section_1 { syn_gocGJ_GapJuncCML_A[1] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[7].Section_1 { syn_gocGJ_GapJuncCML_B[1] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[1].vpeer, a_simple_cellPop[7].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[1].vpeer, a_simple_cellPop[0].Section_1.v(0.5)")

        # Elect Connection 2: cell 0, seg 1 (0.5) [0.5 on a_simple_cellPop[0].Section_1] -> cell 8, seg 1 (0.5) [0.5 on a_simple_cellPop[8].Section_1], weight: 1.0
        h("a_simple_cellPop[0].Section_1 { syn_gocGJ_GapJuncCML_A[2] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[8].Section_1 { syn_gocGJ_GapJuncCML_B[2] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[2].vpeer, a_simple_cellPop[8].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[2].vpeer, a_simple_cellPop[0].Section_1.v(0.5)")

        # Elect Connection 3: cell 0, seg 1 (0.5) [0.5 on a_simple_cellPop[0].Section_1] -> cell 9, seg 1 (0.5) [0.5 on a_simple_cellPop[9].Section_1], weight: 1.0
        h("a_simple_cellPop[0].Section_1 { syn_gocGJ_GapJuncCML_A[3] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[9].Section_1 { syn_gocGJ_GapJuncCML_B[3] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[3].vpeer, a_simple_cellPop[9].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[3].vpeer, a_simple_cellPop[0].Section_1.v(0.5)")

        # Elect Connection 4: cell 1, seg 1 (0.5) [0.5 on a_simple_cellPop[1].Section_1] -> cell 4, seg 1 (0.5) [0.5 on a_simple_cellPop[4].Section_1], weight: 1.0
        h("a_simple_cellPop[1].Section_1 { syn_gocGJ_GapJuncCML_A[4] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[4].Section_1 { syn_gocGJ_GapJuncCML_B[4] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[4].vpeer, a_simple_cellPop[4].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[4].vpeer, a_simple_cellPop[1].Section_1.v(0.5)")

        # Elect Connection 5: cell 1, seg 1 (0.5) [0.5 on a_simple_cellPop[1].Section_1] -> cell 8, seg 1 (0.5) [0.5 on a_simple_cellPop[8].Section_1], weight: 1.0
        h("a_simple_cellPop[1].Section_1 { syn_gocGJ_GapJuncCML_A[5] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[8].Section_1 { syn_gocGJ_GapJuncCML_B[5] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[5].vpeer, a_simple_cellPop[8].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[5].vpeer, a_simple_cellPop[1].Section_1.v(0.5)")

        # Elect Connection 6: cell 2, seg 1 (0.5) [0.5 on a_simple_cellPop[2].Section_1] -> cell 8, seg 1 (0.5) [0.5 on a_simple_cellPop[8].Section_1], weight: 1.0
        h("a_simple_cellPop[2].Section_1 { syn_gocGJ_GapJuncCML_A[6] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[8].Section_1 { syn_gocGJ_GapJuncCML_B[6] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[6].vpeer, a_simple_cellPop[8].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[6].vpeer, a_simple_cellPop[2].Section_1.v(0.5)")

        # Elect Connection 7: cell 3, seg 1 (0.5) [0.5 on a_simple_cellPop[3].Section_1] -> cell 9, seg 1 (0.5) [0.5 on a_simple_cellPop[9].Section_1], weight: 1.0
        h("a_simple_cellPop[3].Section_1 { syn_gocGJ_GapJuncCML_A[7] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[9].Section_1 { syn_gocGJ_GapJuncCML_B[7] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[7].vpeer, a_simple_cellPop[9].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[7].vpeer, a_simple_cellPop[3].Section_1.v(0.5)")

        # Elect Connection 8: cell 4, seg 1 (0.5) [0.5 on a_simple_cellPop[4].Section_1] -> cell 8, seg 1 (0.5) [0.5 on a_simple_cellPop[8].Section_1], weight: 1.0
        h("a_simple_cellPop[4].Section_1 { syn_gocGJ_GapJuncCML_A[8] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[8].Section_1 { syn_gocGJ_GapJuncCML_B[8] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[8].vpeer, a_simple_cellPop[8].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[8].vpeer, a_simple_cellPop[4].Section_1.v(0.5)")

        # Elect Connection 9: cell 5, seg 1 (0.5) [0.5 on a_simple_cellPop[5].Section_1] -> cell 6, seg 1 (0.5) [0.5 on a_simple_cellPop[6].Section_1], weight: 1.0
        h("a_simple_cellPop[5].Section_1 { syn_gocGJ_GapJuncCML_A[9] = new GapJuncCML(0.5) }")
        h("a_simple_cellPop[6].Section_1 { syn_gocGJ_GapJuncCML_B[9] = new GapJuncCML(0.5) }")
        h("setpointer syn_gocGJ_GapJuncCML_A[9].vpeer, a_simple_cellPop[6].Section_1.v(0.5)")
        h("setpointer syn_gocGJ_GapJuncCML_B[9].vpeer, a_simple_cellPop[5].Section_1.v(0.5)")

        trec = h.Vector()
        trec.record(h._ref_t)

        h.tstop = tstop

        h.dt = dt

        h.steps_per_ms = 1/h.dt



        # ######################   File to save: sim_gocnet.v.dat (Volts_file)
        # Column: simple_cellPop/0/simple_cell/v
        h(' objectvar v_0_Volts_file ')
        h(' { v_0_Volts_file = new Vector() } ')
        h(' { v_0_Volts_file.record(&a_simple_cellPop[0].Soma.v(0.5)) } ')
        h.v_0_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)
        # Column: simple_cellPop/1/simple_cell/v
        h(' objectvar v_1_Volts_file ')
        h(' { v_1_Volts_file = new Vector() } ')
        h(' { v_1_Volts_file.record(&a_simple_cellPop[1].Soma.v(0.5)) } ')
        h.v_1_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)
        # Column: simple_cellPop/2/simple_cell/v
        h(' objectvar v_2_Volts_file ')
        h(' { v_2_Volts_file = new Vector() } ')
        h(' { v_2_Volts_file.record(&a_simple_cellPop[2].Soma.v(0.5)) } ')
        h.v_2_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)
        # Column: simple_cellPop/3/simple_cell/v
        h(' objectvar v_3_Volts_file ')
        h(' { v_3_Volts_file = new Vector() } ')
        h(' { v_3_Volts_file.record(&a_simple_cellPop[3].Soma.v(0.5)) } ')
        h.v_3_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)
        # Column: simple_cellPop/4/simple_cell/v
        h(' objectvar v_4_Volts_file ')
        h(' { v_4_Volts_file = new Vector() } ')
        h(' { v_4_Volts_file.record(&a_simple_cellPop[4].Soma.v(0.5)) } ')
        h.v_4_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)
        # Column: simple_cellPop/5/simple_cell/v
        h(' objectvar v_5_Volts_file ')
        h(' { v_5_Volts_file = new Vector() } ')
        h(' { v_5_Volts_file.record(&a_simple_cellPop[5].Soma.v(0.5)) } ')
        h.v_5_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)
        # Column: simple_cellPop/6/simple_cell/v
        h(' objectvar v_6_Volts_file ')
        h(' { v_6_Volts_file = new Vector() } ')
        h(' { v_6_Volts_file.record(&a_simple_cellPop[6].Soma.v(0.5)) } ')
        h.v_6_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)
        # Column: simple_cellPop/7/simple_cell/v
        h(' objectvar v_7_Volts_file ')
        h(' { v_7_Volts_file = new Vector() } ')
        h(' { v_7_Volts_file.record(&a_simple_cellPop[7].Soma.v(0.5)) } ')
        h.v_7_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)
        # Column: simple_cellPop/8/simple_cell/v
        h(' objectvar v_8_Volts_file ')
        h(' { v_8_Volts_file = new Vector() } ')
        h(' { v_8_Volts_file.record(&a_simple_cellPop[8].Soma.v(0.5)) } ')
        h.v_8_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)
        # Column: simple_cellPop/9/simple_cell/v
        h(' objectvar v_9_Volts_file ')
        h(' { v_9_Volts_file = new Vector() } ')
        h(' { v_9_Volts_file.record(&a_simple_cellPop[9].Soma.v(0.5)) } ')
        h.v_9_Volts_file.resize((h.tstop * h.steps_per_ms) + 1)

        # ######################   File to save: sim_gocnet.v.spikes (Events_file)
        h(' objectvar spiketimes_Events_file, t_spiketimes_Events_file ')
        h(' { spiketimes_Events_file = new Vector() } ')
        h(' { t_spiketimes_Events_file = new Vector() } ')
        h(' objref netConnSpike_Events_file, nil ')
        # Column: simple_cellPop/0/simple_cell (0) a_simple_cellPop[0].Soma
        h(' a_simple_cellPop[0].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 0) } ')
        # Column: simple_cellPop/1/simple_cell (1) a_simple_cellPop[1].Soma
        h(' a_simple_cellPop[1].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 1) } ')
        # Column: simple_cellPop/2/simple_cell (2) a_simple_cellPop[2].Soma
        h(' a_simple_cellPop[2].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 2) } ')
        # Column: simple_cellPop/3/simple_cell (3) a_simple_cellPop[3].Soma
        h(' a_simple_cellPop[3].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 3) } ')
        # Column: simple_cellPop/4/simple_cell (4) a_simple_cellPop[4].Soma
        h(' a_simple_cellPop[4].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 4) } ')
        # Column: simple_cellPop/5/simple_cell (5) a_simple_cellPop[5].Soma
        h(' a_simple_cellPop[5].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 5) } ')
        # Column: simple_cellPop/6/simple_cell (6) a_simple_cellPop[6].Soma
        h(' a_simple_cellPop[6].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 6) } ')
        # Column: simple_cellPop/7/simple_cell (7) a_simple_cellPop[7].Soma
        h(' a_simple_cellPop[7].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 7) } ')
        # Column: simple_cellPop/8/simple_cell (8) a_simple_cellPop[8].Soma
        h(' a_simple_cellPop[8].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 8) } ')
        # Column: simple_cellPop/9/simple_cell (9) a_simple_cellPop[9].Soma
        h(' a_simple_cellPop[9].Soma { netConnSpike_Events_file = new NetCon(&v(0.5), nil, 0.0, 0, 1) } ')
        h(' { netConnSpike_Events_file.record(t_spiketimes_Events_file, spiketimes_Events_file, 9) } ')

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

        # ######################   File to save: sim_gocnet.v.dat (Volts_file)
        py_v_0_Volts_file = [ float(x  / 1000.0) for x in h.v_0_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage
        py_v_1_Volts_file = [ float(x  / 1000.0) for x in h.v_1_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage
        py_v_2_Volts_file = [ float(x  / 1000.0) for x in h.v_2_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage
        py_v_3_Volts_file = [ float(x  / 1000.0) for x in h.v_3_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage
        py_v_4_Volts_file = [ float(x  / 1000.0) for x in h.v_4_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage
        py_v_5_Volts_file = [ float(x  / 1000.0) for x in h.v_5_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage
        py_v_6_Volts_file = [ float(x  / 1000.0) for x in h.v_6_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage
        py_v_7_Volts_file = [ float(x  / 1000.0) for x in h.v_7_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage
        py_v_8_Volts_file = [ float(x  / 1000.0) for x in h.v_8_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage
        py_v_9_Volts_file = [ float(x  / 1000.0) for x in h.v_9_Volts_file.to_python() ]  # Convert to Python list for speed, variable has dim: voltage

        f_Volts_file_f2 = open('sim_gocnet.v.dat', 'w')
        num_points = len(py_v_time)  # Simulation may have been stopped before tstop...

        for i in range(num_points):
            f_Volts_file_f2.write('%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t\n' % (py_v_time[i], py_v_0_Volts_file[i], py_v_1_Volts_file[i], py_v_2_Volts_file[i], py_v_3_Volts_file[i], py_v_4_Volts_file[i], py_v_5_Volts_file[i], py_v_6_Volts_file[i], py_v_7_Volts_file[i], py_v_8_Volts_file[i], py_v_9_Volts_file[i], ))
        f_Volts_file_f2.close()
        print("Saved data to: sim_gocnet.v.dat")

        # ######################   File to save: sim_gocnet.v.spikes (Events_file)

        f_Events_file_f2 = open('sim_gocnet.v.spikes', 'w')
        h(' objref netConnSpike_Events_file ')
        spike_ids = h.spiketimes_Events_file.to_python()  
        spike_times = h.t_spiketimes_Events_file.to_python()
        for i, id in enumerate(spike_ids):
            # Saving in format: ID_TIME
            f_Events_file_f2.write("%i\t%s\n"%(id,spike_times[i]/1000.0))
        f_Events_file_f2.close()
        print("Saved data to: sim_gocnet.v.spikes")

        save_end = time.time()
        save_time = save_end - self.sim_end
        print("Finished saving results in %f seconds"%(save_time))

        print("Done")

        quit()


if __name__ == '__main__':

    ns = NeuronSimulation(tstop=200, dt=0.025, seed=12345)

    ns.run()

