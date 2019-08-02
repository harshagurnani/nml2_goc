TITLE Mod file for component: Component(id=CaClamp type=caClamp)

COMMENT

    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.4.3
         org.neuroml.model   v1.4.3
         jLEMS               v0.9.8.1

ENDCOMMENT

NEURON {
    SUFFIX CaClamp
    USEION ca READ cai, cao, ica WRITE cai VALENCE 2
    RANGE cai
    RANGE cao
    GLOBAL initialConcentration
    GLOBAL initialExtConcentration
    RANGE conc0                             : parameter
    RANGE conc1                             : parameter
    RANGE delay                             : parameter
    RANGE duration                          : parameter
    
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
    surfaceArea (um2)
    iCa (nA)
    initialConcentration (mM)
    initialExtConcentration (mM)
    
    conc0 = 5.0E-5 (mM)
    conc1 = 0.005 (mM)
    delay = 200 (ms)
    duration = 200 (ms)
}

ASSIGNED {
    cai (mM)
    cao (mM)
    ica (mA/cm2)
    diam (um)
    area (um2)
    
}

STATE {
    concentration (mM)
    extConcentration (mM)
    
}

INITIAL {
    initialConcentration = cai
    initialExtConcentration = cao
    rates()
    rates() ? To ensure correct initialisation.
    
    concentration = initialConcentration
    
    extConcentration = initialExtConcentration
    
}

BREAKPOINT {
    
    rates()
    if (t <=  delay) {
        concentration = conc0 ? standard OnCondition
    }
    
    if (t >=  delay  && t <=  duration  +  delay) {
        concentration = conc1 ? standard OnCondition
    }
    
    if (t >=  duration  +  delay) {
        concentration = conc0 ? standard OnCondition
    }
    
    
    cai = concentration
    
    
}

PROCEDURE rates() {
    
    surfaceArea = area   : surfaceArea has units (um2), area (built in to NEURON) is in um^2...
    
    iCa = -1 * (0.01) * ica * surfaceArea :   iCa has units (nA) ; ica (built in to NEURON) has units (mA/cm2)...
    
    
     
    
}

