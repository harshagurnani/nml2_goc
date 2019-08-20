TITLE Mod file for component: Component(id=MFGoCSyn type=expThreeSynapse)

COMMENT

    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.5.4
         org.neuroml.model   v1.5.4
         jLEMS               v0.9.9.1

ENDCOMMENT

NEURON {
    POINT_PROCESS MFGoCSyn
    RANGE tauRise                           : parameter
    RANGE tauDecay1                         : parameter
    RANGE tauDecay2                         : parameter
    RANGE peakTime1                         : parameter
    RANGE waveformFactor1                   : parameter
    RANGE peakTime2                         : parameter
    RANGE waveformFactor2                   : parameter
    RANGE gbase1                            : parameter
    RANGE gbase2                            : parameter
    RANGE erev                              : parameter
    
    RANGE g                                 : exposure
    
    RANGE i                                 : exposure
    
    
    NONSPECIFIC_CURRENT i 
    
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
    
    tauRise = 0.1 (ms)
    tauDecay1 = 0.7 (ms)
    tauDecay2 = 3.5 (ms)
    peakTime1 = 0.22702286 (ms)
    waveformFactor1 = 1.6136022 
    peakTime2 = 0.3659917 (ms)
    waveformFactor2 = 1.1428859 
    gbase1 = 7.0E-4 (uS)
    gbase2 = 2.0E-4 (uS)
    erev = 0 (mV)
}

ASSIGNED {
    ? Standard Assigned variables with baseSynapse
    v (mV)
    celsius (degC)
    temperature (K)
    
    g (uS)                                 : derived variable
    
    i (nA)                                 : derived variable
    rate_A (/ms)
    rate_B (/ms)
    rate_C (/ms)
    
}

STATE {
    A  
    B  
    C  
    
}

INITIAL {
    temperature = celsius + 273.15
    
    rates()
    rates() ? To ensure correct initialisation.
    
    A = 0
    
    B = 0
    
    C = 0
    
}

BREAKPOINT {
    
    SOLVE states METHOD cnexp
    
    
}

NET_RECEIVE(weight) {
    
    : paramMappings . : {MFGoCSyn={A=A, B=B, C=C, tauRise=tauRise, tauDecay1=tauDecay1, tauDecay2=tauDecay2, peakTime1=peakTime1, waveformFactor1=waveformFactor1, peakTime2=peakTime2, waveformFactor2=waveformFactor2, gbase1=gbase1, gbase2=gbase2, erev=erev, g=g, i=i}}
    : state_discontinuity(A, A  + (  gbase1  *weight *  waveformFactor1  +   gbase2  *weight*  waveformFactor2   )/(  gbase1  +  gbase2  )) : From MFGoCSyn
    A = A  + (  gbase1  *weight *  waveformFactor1  +   gbase2  *weight*  waveformFactor2   )/(  gbase1  +  gbase2  ) : From MFGoCSyn
    
    : paramMappings . : {MFGoCSyn={A=A, B=B, C=C, tauRise=tauRise, tauDecay1=tauDecay1, tauDecay2=tauDecay2, peakTime1=peakTime1, waveformFactor1=waveformFactor1, peakTime2=peakTime2, waveformFactor2=waveformFactor2, gbase1=gbase1, gbase2=gbase2, erev=erev, g=g, i=i}}
    : state_discontinuity(B, B  + (weight *   waveformFactor1  )) : From MFGoCSyn
    B = B  + (weight *   waveformFactor1  ) : From MFGoCSyn
    
    : paramMappings . : {MFGoCSyn={A=A, B=B, C=C, tauRise=tauRise, tauDecay1=tauDecay1, tauDecay2=tauDecay2, peakTime1=peakTime1, waveformFactor1=waveformFactor1, peakTime2=peakTime2, waveformFactor2=waveformFactor2, gbase1=gbase1, gbase2=gbase2, erev=erev, g=g, i=i}}
    : state_discontinuity(C, C  + (weight *   waveformFactor2  )) : From MFGoCSyn
    C = C  + (weight *   waveformFactor2  ) : From MFGoCSyn
    
}

DERIVATIVE states {
    rates()
    A' = rate_A 
    B' = rate_B 
    C' = rate_C 
    
}

PROCEDURE rates() {
    
    g = gbase1  *(  B   -   A  ) +   gbase2  *(  C  -  A  ) ? evaluable
    i = -1 * g  * (  erev   - v) ? evaluable
    rate_A = -  A   /  tauRise ? Note units of all quantities used here need to be consistent!
    rate_B = -  B   /  tauDecay1 ? Note units of all quantities used here need to be consistent!
    rate_C = -  C   /  tauDecay2 ? Note units of all quantities used here need to be consistent!
    
     
    
}

