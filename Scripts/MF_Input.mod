TITLE Mod file for component: Component(id=MF_Input type=spikeGenerator)

COMMENT

    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.5.4
         org.neuroml.model   v1.5.4
         jLEMS               v0.9.9.1

ENDCOMMENT

NEURON {
    POINT_PROCESS MF_Input
    RANGE period                            : parameter
    
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
    
    period = 150 (ms)
}

ASSIGNED {
    rate_tsince (ms/ms)
    
}

STATE {
    tsince (ms) 
    
}

INITIAL {
    rates()
    rates() ? To ensure correct initialisation.
    
    tsince = 0
    
    net_send(0, 1) : go to NET_RECEIVE block, flag 1, for initial state
    
}

BREAKPOINT {
    
    SOLVE states METHOD cnexp
    
    
}

NET_RECEIVE(flag) {
    
    LOCAL weight
    
    
    if (flag == 1) { : Setting watch for top level OnCondition...
        WATCH (tsince  >  period) 1000
    }
    if (flag == 1000) {
    
        tsince = 0
    
        net_event(t)
        WATCH (tsince  >  period) 1000
    
    }
    
}

DERIVATIVE states {
    rates()
    tsince' = rate_tsince 
    
}

PROCEDURE rates() {
    
    rate_tsince = 1 ? Note units of all quantities used here need to be consistent!
    
     
    
}

