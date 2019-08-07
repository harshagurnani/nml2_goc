TITLE Mod file for component: Component(id=GolgiCaHVA type=ionChannelHH)

COMMENT

    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.5.4
         org.neuroml.model   v1.5.4
         jLEMS               v0.9.9.1

ENDCOMMENT

NEURON {
    SUFFIX GolgiCaHVA
    USEION ca READ eca WRITE ica VALENCE 2 ? Assuming valence = 2 (Ca ion); TODO check this!!
    
    RANGE gion                           
    RANGE gmax                              : Will be changed when ion channel mechanism placed on cell!
    RANGE conductance                       : parameter
    
    RANGE g                                 : exposure
    
    RANGE fopen                             : exposure
    RANGE s_instances                       : parameter
    
    RANGE s_alpha                           : exposure
    
    RANGE s_beta                            : exposure
    
    RANGE s_tau                             : exposure
    
    RANGE s_inf                             : exposure
    
    RANGE s_rateScale                       : exposure
    
    RANGE s_fcond                           : exposure
    RANGE s_forwardRate_rate                : parameter
    RANGE s_forwardRate_midpoint            : parameter
    RANGE s_forwardRate_scale               : parameter
    
    RANGE s_forwardRate_r                   : exposure
    RANGE s_reverseRate_rate                : parameter
    RANGE s_reverseRate_midpoint            : parameter
    RANGE s_reverseRate_scale               : parameter
    
    RANGE s_reverseRate_r                   : exposure
    RANGE s_timeCourse_TIME_SCALE           : parameter
    
    RANGE s_timeCourse_t                    : exposure
    RANGE s_q10Settings_q10Factor           : parameter
    RANGE s_q10Settings_experimentalTemp    : parameter
    RANGE s_q10Settings_TENDEGREES          : parameter
    
    RANGE s_q10Settings_q10                 : exposure
    RANGE u_instances                       : parameter
    
    RANGE u_alpha                           : exposure
    
    RANGE u_beta                            : exposure
    
    RANGE u_tau                             : exposure
    
    RANGE u_inf                             : exposure
    
    RANGE u_rateScale                       : exposure
    
    RANGE u_fcond                           : exposure
    RANGE u_forwardRate_rate                : parameter
    RANGE u_forwardRate_midpoint            : parameter
    RANGE u_forwardRate_scale               : parameter
    
    RANGE u_forwardRate_r                   : exposure
    RANGE u_reverseRate_rate                : parameter
    RANGE u_reverseRate_midpoint            : parameter
    RANGE u_reverseRate_scale               : parameter
    
    RANGE u_reverseRate_r                   : exposure
    RANGE u_q10Settings_q10Factor           : parameter
    RANGE u_q10Settings_experimentalTemp    : parameter
    RANGE u_q10Settings_TENDEGREES          : parameter
    
    RANGE u_q10Settings_q10                 : exposure
    RANGE s_tauUnscaled                     : derived variable
    RANGE conductanceScale                  : derived variable
    RANGE fopen0                            : derived variable
    
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
    
    gmax = 0  (S/cm2)                       : Will be changed when ion channel mechanism placed on cell!
    
    conductance = 1.0E-5 (uS)
    s_instances = 2 
    s_forwardRate_rate = 0.049440004 (kHz)
    s_forwardRate_midpoint = -29.06 (mV)
    s_forwardRate_scale = 15.873017 (mV)
    s_reverseRate_rate = 0.08298001 (kHz)
    s_reverseRate_midpoint = -18.66 (mV)
    s_reverseRate_scale = -25.641 (mV)
    s_timeCourse_TIME_SCALE = 1 (ms)
    s_q10Settings_q10Factor = 3 
    s_q10Settings_experimentalTemp = 293.15 (K)
    s_q10Settings_TENDEGREES = 10 (K)
    u_instances = 1 
    u_forwardRate_rate = 0.0013000001 (kHz)
    u_forwardRate_midpoint = -48 (mV)
    u_forwardRate_scale = -18.1832 (mV)
    u_reverseRate_rate = 0.0013000001 (kHz)
    u_reverseRate_midpoint = -48 (mV)
    u_reverseRate_scale = 83.33 (mV)
    u_q10Settings_q10Factor = 3 
    u_q10Settings_experimentalTemp = 293.15 (K)
    u_q10Settings_TENDEGREES = 10 (K)
}

ASSIGNED {
    
    gion   (S/cm2)                          : Transient conductance density of the channel? Standard Assigned variables with ionChannel
    v (mV)
    celsius (degC)
    temperature (K)
    eca (mV)
    ica (mA/cm2)
    
    
    s_forwardRate_r (kHz)                  : derived variable
    
    s_reverseRate_r (kHz)                  : derived variable
    
    s_timeCourse_t (ms)                    : conditional derived var...
    
    s_q10Settings_q10                      : derived variable
    
    s_rateScale                            : derived variable
    
    s_alpha (kHz)                          : derived variable
    
    s_beta (kHz)                           : derived variable
    
    s_fcond                                : derived variable
    
    s_inf                                  : derived variable
    
    s_tauUnscaled (ms)                     : derived variable
    
    s_tau (ms)                             : derived variable
    
    u_forwardRate_r (kHz)                  : derived variable
    
    u_reverseRate_r (kHz)                  : derived variable
    
    u_q10Settings_q10                      : derived variable
    
    u_rateScale                            : derived variable
    
    u_alpha (kHz)                          : derived variable
    
    u_beta (kHz)                           : derived variable
    
    u_fcond                                : derived variable
    
    u_inf                                  : derived variable
    
    u_tau (ms)                             : derived variable
    
    conductanceScale                       : derived variable
    
    fopen0                                 : derived variable
    
    fopen                                  : derived variable
    
    g (uS)                                 : derived variable
    rate_s_q (/ms)
    rate_u_q (/ms)
    
}

STATE {
    s_q  
    u_q  
    
}

INITIAL {
    temperature = celsius + 273.15
    
    rates()
    rates() ? To ensure correct initialisation.
    
    s_q = s_inf
    
    u_q = u_inf
    
}

BREAKPOINT {
    
    SOLVE states METHOD cnexp
    
    ? DerivedVariable is based on path: conductanceScaling[*]/factor, on: Component(id=GolgiCaHVA type=ionChannelHH), from conductanceScaling; null
    ? Path not present in component, using factor: 1
    
    conductanceScale = 1 
    
    ? DerivedVariable is based on path: gates[*]/fcond, on: Component(id=GolgiCaHVA type=ionChannelHH), from gates; Component(id=s type=gateHHratesTau)
    ? multiply applied to all instances of fcond in: <gates> ([Component(id=s type=gateHHratesTau), Component(id=u type=gateHHrates)]))
    fopen0 = s_fcond * u_fcond ? path based, prefix = 
    
    fopen = conductanceScale  *  fopen0 ? evaluable
    g = conductance  *  fopen ? evaluable
    gion = gmax * fopen 
    
    ica = gion * (v - eca)
    
}

DERIVATIVE states {
    rates()
    s_q' = rate_s_q 
    u_q' = rate_u_q 
    
}

PROCEDURE rates() {
    
    s_forwardRate_r = s_forwardRate_rate  * exp((v -  s_forwardRate_midpoint )/ s_forwardRate_scale ) ? evaluable
    s_reverseRate_r = s_reverseRate_rate  * exp((v -  s_reverseRate_midpoint )/ s_reverseRate_scale ) ? evaluable
    if (1/( s_alpha  +  s_beta ) > ( 1000 ))  { 
        s_timeCourse_t = 1000 *  s_timeCourse_TIME_SCALE ? evaluable cdv
    } else if (1/( s_alpha  +  s_beta )  < ( 0.1 ))  { 
        s_timeCourse_t = 0.1 *  s_timeCourse_TIME_SCALE ? evaluable cdv
    } else  { 
        s_timeCourse_t = 1/( s_alpha  +  s_beta ) ? evaluable cdv
    }
    
    s_q10Settings_q10 = s_q10Settings_q10Factor ^((temperature -  s_q10Settings_experimentalTemp )/ s_q10Settings_TENDEGREES ) ? evaluable
    ? DerivedVariable is based on path: q10Settings[*]/q10, on: Component(id=s type=gateHHratesTau), from q10Settings; Component(id=null type=q10ExpTemp)
    ? multiply applied to all instances of q10 in: <q10Settings> ([Component(id=null type=q10ExpTemp)]))
    s_rateScale = s_q10Settings_q10 ? path based, prefix = s_
    
    ? DerivedVariable is based on path: forwardRate/r, on: Component(id=s type=gateHHratesTau), from forwardRate; Component(id=null type=HHExpRate)
    s_alpha = s_forwardRate_r ? path based, prefix = s_
    
    ? DerivedVariable is based on path: reverseRate/r, on: Component(id=s type=gateHHratesTau), from reverseRate; Component(id=null type=HHExpRate)
    s_beta = s_reverseRate_r ? path based, prefix = s_
    
    s_fcond = s_q ^ s_instances ? evaluable
    s_inf = s_alpha /( s_alpha + s_beta ) ? evaluable
    ? DerivedVariable is based on path: timeCourse/t, on: Component(id=s type=gateHHratesTau), from timeCourse; Component(id=null type=Golgi_CaHVA_stau)
    s_tauUnscaled = s_timeCourse_t ? path based, prefix = s_
    
    s_tau = s_tauUnscaled  /  s_rateScale ? evaluable
    u_forwardRate_r = u_forwardRate_rate  * exp((v -  u_forwardRate_midpoint )/ u_forwardRate_scale ) ? evaluable
    u_reverseRate_r = u_reverseRate_rate  * exp((v -  u_reverseRate_midpoint )/ u_reverseRate_scale ) ? evaluable
    u_q10Settings_q10 = u_q10Settings_q10Factor ^((temperature -  u_q10Settings_experimentalTemp )/ u_q10Settings_TENDEGREES ) ? evaluable
    ? DerivedVariable is based on path: q10Settings[*]/q10, on: Component(id=u type=gateHHrates), from q10Settings; Component(id=null type=q10ExpTemp)
    ? multiply applied to all instances of q10 in: <q10Settings> ([Component(id=null type=q10ExpTemp)]))
    u_rateScale = u_q10Settings_q10 ? path based, prefix = u_
    
    ? DerivedVariable is based on path: forwardRate/r, on: Component(id=u type=gateHHrates), from forwardRate; Component(id=null type=HHExpRate)
    u_alpha = u_forwardRate_r ? path based, prefix = u_
    
    ? DerivedVariable is based on path: reverseRate/r, on: Component(id=u type=gateHHrates), from reverseRate; Component(id=null type=HHExpRate)
    u_beta = u_reverseRate_r ? path based, prefix = u_
    
    u_fcond = u_q ^ u_instances ? evaluable
    u_inf = u_alpha /( u_alpha + u_beta ) ? evaluable
    u_tau = 1/(( u_alpha + u_beta ) *  u_rateScale ) ? evaluable
    
     
    
     
    
     
    
     
    
     
    
     
    
     
    
     
    
     
    
     
    
     
    
     
    rate_s_q = ( s_inf  -  s_q ) /  s_tau ? Note units of all quantities used here need to be consistent!
    
     
    
     
    
     
    
     
    
     
    rate_u_q = ( u_inf  -  u_q ) /  u_tau ? Note units of all quantities used here need to be consistent!
    
     
    
     
    
     
    
     
    
}

