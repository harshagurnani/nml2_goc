TITLE Mod file for component: Component(id=MF_Burst type=transientPoissonFiringSynapse)

COMMENT

    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.5.4
         org.neuroml.model   v1.5.4
         jLEMS               v0.9.9.1

ENDCOMMENT

NEURON {
    POINT_PROCESS MF_Burst
    ELECTRODE_CURRENT i
    RANGE weight                            : property
    RANGE averageRate                       : parameter
    RANGE delay                             : parameter
    RANGE duration                          : parameter
    RANGE averageIsi                        : parameter
    RANGE LONG_TIME                         : parameter
    
    RANGE i                                 : exposure
    RANGE MFGoC_SynMult_tauRise             : parameter
    RANGE MFGoC_SynMult_tauDecay1           : parameter
    RANGE MFGoC_SynMult_tauDecay2           : parameter
    RANGE MFGoC_SynMult_peakTime1           : parameter
    RANGE MFGoC_SynMult_waveformFactor1     : parameter
    RANGE MFGoC_SynMult_peakTime2           : parameter
    RANGE MFGoC_SynMult_waveformFactor2     : parameter
    RANGE MFGoC_SynMult_gbase1              : parameter
    RANGE MFGoC_SynMult_gbase2              : parameter
    RANGE MFGoC_SynMult_erev                : parameter
    
    RANGE MFGoC_SynMult_g                   : exposure
    
    RANGE MFGoC_SynMult_i                   : exposure
    RANGE iSyn                              : derived variable
    : Based on netstim.mod
    THREADSAFE : only true if every instance has its own distinct Random
    POINTER donotuse
}

UNITS {
    
    (nA) = (nanoamp)
    (uA) = (microamp)
    (mA) = (milliamp)
    (A) = (amp)
    (mV) = (millivolt)
    (mS) = (millisiemens)
    (uS) = (microsiemens)
    (molar) = (1/liter)
    (kHz) = (kilohertz)
    (mM) = (millimolar)
    (um) = (micrometer)
    (umol) = (micromole)
    (S) = (siemens)
    
}

PARAMETER {
    
    weight = 1
    averageRate = 0.1 (kHz)
    delay = 2000 (ms)
    duration = 500 (ms)
    averageIsi = 10 (ms)
    LONG_TIME = 3.59999998E15 (ms)
    MFGoC_SynMult_tauRise = 0.1 (ms)
    MFGoC_SynMult_tauDecay1 = 0.7 (ms)
    MFGoC_SynMult_tauDecay2 = 3.5 (ms)
    MFGoC_SynMult_peakTime1 = 0.22702286 (ms)
    MFGoC_SynMult_waveformFactor1 = 1.6136022 
    MFGoC_SynMult_peakTime2 = 0.3659917 (ms)
    MFGoC_SynMult_waveformFactor2 = 1.1428859 
    MFGoC_SynMult_gbase1 = 0.0035 (uS)
    MFGoC_SynMult_gbase2 = 0.001 (uS)
    MFGoC_SynMult_erev = 0 (mV)
}

ASSIGNED {
    v (mV)
    nextIsi (ms)                    : Not a state variable as far as Neuron's concerned...
    isi (ms)                    : Not a state variable as far as Neuron's concerned...
    
    MFGoC_SynMult_g (uS)                   : derived variable
    
    MFGoC_SynMult_i (nA)                   : derived variable
    
    iSyn (nA)                              : derived variable
    
    i (nA)                                 : derived variable
    rate_tsince (ms/ms)
    rate_MFGoC_SynMult_A (/ms)
    rate_MFGoC_SynMult_B (/ms)
    rate_MFGoC_SynMult_C (/ms)
    donotuse
}

STATE {
    tsince (ms) 
    MFGoC_SynMult_A  
    MFGoC_SynMult_B  
    MFGoC_SynMult_C  
    
}

INITIAL {
    rates()
    rates() ? To ensure correct initialisation.
    
    tsince = 0
    
    nextIsi = -  averageIsi  * log(1 - random_float(1))  +  delay
    
    isi = nextIsi
    
    net_send(0, 1) : go to NET_RECEIVE block, flag 1, for initial state
    
    MFGoC_SynMult_A = 0
    
    MFGoC_SynMult_B = 0
    
    MFGoC_SynMult_C = 0
    
}

BREAKPOINT {
    
    SOLVE states METHOD cnexp
    
    
}

NET_RECEIVE(flag) {
    
    LOCAL weight
    
    
    if (flag == 1) { : Setting watch for top level OnCondition...
        WATCH (tsince  >  isi) 1000
    }
    if (flag == 1000) {
    
        tsince = 0
    
        nextIsi = -  averageIsi  * log(1 - random_float(1))
    
        isi = nextIsi  + H(((t+  nextIsi  ) - (  delay  +  duration  ))/  duration  )*  LONG_TIME
    
        : Child: Component(id=MFGoC_SynMult type=expThreeSynapse)
    
        : This child is a synapse; defining weight
        weight = 1
    
        : paramMappings are: {MF_Burst={tsince=tsince, nextIsi=nextIsi, isi=isi, weight=weight, averageRate=averageRate, delay=delay, duration=duration, averageIsi=averageIsi, LONG_TIME=LONG_TIME, i=i, iSyn=iSyn}, MFGoC_SynMult={A=MFGoC_SynMult_A, B=MFGoC_SynMult_B, C=MFGoC_SynMult_C, tauRise=MFGoC_SynMult_tauRise, tauDecay1=MFGoC_SynMult_tauDecay1, tauDecay2=MFGoC_S...
        : state_discontinuity(MFGoC_SynMult_A, MFGoC_SynMult_A  + ( MFGoC_SynMult_gbase1 *weight *  MFGoC_SynMult_waveformFactor1  +  MFGoC_SynMult_gbase2 *weight* MFGoC_SynMult_waveformFactor2  )/( MFGoC_SynMult_gbase1 + MFGoC_SynMult_gbase2 ))
        MFGoC_SynMult_A = MFGoC_SynMult_A  + ( MFGoC_SynMult_gbase1 *weight *  MFGoC_SynMult_waveformFactor1  +  MFGoC_SynMult_gbase2 *weight* MFGoC_SynMult_waveformFactor2  )/( MFGoC_SynMult_gbase1 + MFGoC_SynMult_gbase2 )
    
        : paramMappings are: {MF_Burst={tsince=tsince, nextIsi=nextIsi, isi=isi, weight=weight, averageRate=averageRate, delay=delay, duration=duration, averageIsi=averageIsi, LONG_TIME=LONG_TIME, i=i, iSyn=iSyn}, MFGoC_SynMult={A=MFGoC_SynMult_A, B=MFGoC_SynMult_B, C=MFGoC_SynMult_C, tauRise=MFGoC_SynMult_tauRise, tauDecay1=MFGoC_SynMult_tauDecay1, tauDecay2=MFGoC_S...
        : state_discontinuity(MFGoC_SynMult_B, MFGoC_SynMult_B  + (weight *  MFGoC_SynMult_waveformFactor1 ))
        MFGoC_SynMult_B = MFGoC_SynMult_B  + (weight *  MFGoC_SynMult_waveformFactor1 )
    
        : paramMappings are: {MF_Burst={tsince=tsince, nextIsi=nextIsi, isi=isi, weight=weight, averageRate=averageRate, delay=delay, duration=duration, averageIsi=averageIsi, LONG_TIME=LONG_TIME, i=i, iSyn=iSyn}, MFGoC_SynMult={A=MFGoC_SynMult_A, B=MFGoC_SynMult_B, C=MFGoC_SynMult_C, tauRise=MFGoC_SynMult_tauRise, tauDecay1=MFGoC_SynMult_tauDecay1, tauDecay2=MFGoC_S...
        : state_discontinuity(MFGoC_SynMult_C, MFGoC_SynMult_C  + (weight *  MFGoC_SynMult_waveformFactor2 ))
        MFGoC_SynMult_C = MFGoC_SynMult_C  + (weight *  MFGoC_SynMult_waveformFactor2 )
    
        net_event(t)
        WATCH (tsince  >  isi) 1000
    
    }
    
}

DERIVATIVE states {
    rates()
    tsince' = rate_tsince 
    MFGoC_SynMult_A' = rate_MFGoC_SynMult_A 
    MFGoC_SynMult_B' = rate_MFGoC_SynMult_B 
    MFGoC_SynMult_C' = rate_MFGoC_SynMult_C 
    
}

PROCEDURE rates() {
    
    MFGoC_SynMult_g = MFGoC_SynMult_gbase1 *( MFGoC_SynMult_B  -  MFGoC_SynMult_A ) +  MFGoC_SynMult_gbase2 *( MFGoC_SynMult_C - MFGoC_SynMult_A ) ? evaluable
    MFGoC_SynMult_i = MFGoC_SynMult_g  * ( MFGoC_SynMult_erev  - v) ? evaluable
    ? DerivedVariable is based on path: synapse/i, on: Component(id=MF_Burst type=transientPoissonFiringSynapse), from synapse; Component(id=MFGoC_SynMult type=expThreeSynapse)
    iSyn = MFGoC_SynMult_i ? path based, prefix = 
    
    i = weight  *  iSyn ? evaluable
    rate_tsince = 1 ? Note units of all quantities used here need to be consistent!
    
     
    rate_MFGoC_SynMult_A = - MFGoC_SynMult_A  /  MFGoC_SynMult_tauRise ? Note units of all quantities used here need to be consistent!
    rate_MFGoC_SynMult_B = - MFGoC_SynMult_B  /  MFGoC_SynMult_tauDecay1 ? Note units of all quantities used here need to be consistent!
    rate_MFGoC_SynMult_C = - MFGoC_SynMult_C  /  MFGoC_SynMult_tauDecay2 ? Note units of all quantities used here need to be consistent!
    
     
    
}


: Returns a float between 0 and max; implementation of random() as used in LEMS
FUNCTION random_float(max) {
    
    : This is not ideal, getting an exponential dist random number and then turning back to uniform
    : However this is the easiest what to ensure mod files with random methods fit into NEURON's
    : internal framework for managing internal number generation.
    random_float = exp(-1*erand())*max
    
}

:****************************************************
: Methods copied from netstim.mod in NEURON source

 
PROCEDURE seed(x) {
	set_seed(x)
}

VERBATIM
double nrn_random_pick(void* r);
void* nrn_random_arg(int argpos);
ENDVERBATIM


FUNCTION erand() {
VERBATIM
	if (_p_donotuse) {
		/*
		:Supports separate independent but reproducible streams for
		: each instance. However, the corresponding hoc Random
		: distribution MUST be set to Random.negexp(1)
		*/
		_lerand = nrn_random_pick(_p_donotuse);
	}else{
		/* only can be used in main thread */
		if (_nt != nrn_threads) {
           hoc_execerror("multithread random in NetStim"," only via hoc Random");
		}
ENDVERBATIM
		: the old standby. Cannot use if reproducible parallel sim
		: independent of nhost or which host this instance is on
		: is desired, since each instance on this cpu draws from
		: the same stream
		erand = exprand(1)
VERBATIM
	}
ENDVERBATIM
}

PROCEDURE noiseFromRandom() {
VERBATIM
 {
	void** pv = (void**)(&_p_donotuse);
	if (ifarg(1)) {
		*pv = nrn_random_arg(1);
	}else{
		*pv = (void*)0;
	}
 }
ENDVERBATIM
}

: End of methods copied from netstim.mod in NEURON source
:****************************************************


: The Heaviside step function
FUNCTION H(x) {
    
    if (x < 0) { H = 0 }
    else if (x > 0) { H = 1 }
    else { H = 0.5 }
    
}

