#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _GolgiHCN2f_reg();
extern void _GolgiHCN2s_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," GolgiHCN2f.mod");
fprintf(stderr," GolgiHCN2s.mod");
fprintf(stderr, "\n");
    }
_GolgiHCN2f_reg();
_GolgiHCN2s_reg();
}
