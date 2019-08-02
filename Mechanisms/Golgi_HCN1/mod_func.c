#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _GolgiHCN1f_reg();
extern void _GolgiHCN1s_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," GolgiHCN1f.mod");
fprintf(stderr," GolgiHCN1s.mod");
fprintf(stderr, "\n");
    }
_GolgiHCN1f_reg();
_GolgiHCN1s_reg();
}
