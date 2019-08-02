#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _CaClamp_reg();
extern void _Golgi_BK_reg();
extern void _Golgi_CALC_reg();
extern void _Golgi_CALC_ca2_reg();
extern void _Golgi_Ca_HVA_reg();
extern void _Golgi_Ca_LVA_reg();
extern void _Golgi_KA_reg();
extern void _Golgi_KM_reg();
extern void _Golgi_KV_reg();
extern void _Golgi_Na_reg();
extern void _Golgi_NaP_reg();
extern void _Golgi_NaR_reg();
extern void _Golgi_SK2_reg();
extern void _Golgi_hcn1_reg();
extern void _Golgi_hcn2_reg();
extern void _Golgi_lkg_reg();
extern void _LeakConductance_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," CaClamp.mod");
fprintf(stderr," Golgi_BK.mod");
fprintf(stderr," Golgi_CALC.mod");
fprintf(stderr," Golgi_CALC_ca2.mod");
fprintf(stderr," Golgi_Ca_HVA.mod");
fprintf(stderr," Golgi_Ca_LVA.mod");
fprintf(stderr," Golgi_KA.mod");
fprintf(stderr," Golgi_KM.mod");
fprintf(stderr," Golgi_KV.mod");
fprintf(stderr," Golgi_Na.mod");
fprintf(stderr," Golgi_NaP.mod");
fprintf(stderr," Golgi_NaR.mod");
fprintf(stderr," Golgi_SK2.mod");
fprintf(stderr," Golgi_hcn1.mod");
fprintf(stderr," Golgi_hcn2.mod");
fprintf(stderr," Golgi_lkg.mod");
fprintf(stderr," LeakConductance.mod");
fprintf(stderr, "\n");
    }
_CaClamp_reg();
_Golgi_BK_reg();
_Golgi_CALC_reg();
_Golgi_CALC_ca2_reg();
_Golgi_Ca_HVA_reg();
_Golgi_Ca_LVA_reg();
_Golgi_KA_reg();
_Golgi_KM_reg();
_Golgi_KV_reg();
_Golgi_Na_reg();
_Golgi_NaP_reg();
_Golgi_NaR_reg();
_Golgi_SK2_reg();
_Golgi_hcn1_reg();
_Golgi_hcn2_reg();
_Golgi_lkg_reg();
_LeakConductance_reg();
}
