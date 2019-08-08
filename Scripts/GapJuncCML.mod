TITLE Mod file for component: Component(id=GapJuncCML type=gapJunction)

COMMENT

    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.5.4
         org.neuroml.model   v1.5.4
         jLEMS               v0.9.9.1

ENDCOMMENT

NEURON {
    POINT_PROCESS GapJuncCML
    RANGE weight                            : property
    RANGE conductance                       : parameter
    
    RANGE i                                 : exposure
    
    
    NONSPECIFIC_CURRENT i 
    POINTER vpeer: derived variable as pointer...
    
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
    conductance = 5.0E-4 (uS)
}

ASSIGNED {
    ? Standard Assigned variables with baseSynapse
    v (mV)
    celsius (degC)
    temperature (K)
    
    vpeer (mV)                             : derived variable
    
    i : no units???                        : derived variable
    
}

STATE {
    
}

INITIAL {
    temperature = celsius + 273.15
    
    rates()
    rates() ? To ensure correct initialisation.
    
}

BREAKPOINT {
    
    rates()
    
}

PROCEDURE rates() {
    
    ? DerivedVariable is based on path: peer/v, on: Component(id=GapJuncCML type=gapJunction), from peer; null
    ? Derived variable: vpeer; its value will be set by a pointer...
    
    i = -1 * weight  *  conductance  * (vpeer - v) ? evaluable
    
     
    
}

