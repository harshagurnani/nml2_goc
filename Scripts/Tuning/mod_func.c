#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _AlphaSyn_reg();
extern void _GJ_0_reg();
extern void _GapJuncCML_reg();
extern void _GolgiBK_reg();
extern void _GolgiCaHVA_reg();
extern void _GolgiCaLVA_reg();
extern void _GolgiHCN1f_reg();
extern void _GolgiHCN1s_reg();
extern void _GolgiHCN2f_reg();
extern void _GolgiHCN2s_reg();
extern void _GolgiKA_reg();
extern void _GolgiKM_reg();
extern void _GolgiKV_reg();
extern void _GolgiNa_reg();
extern void _GolgiNaP_reg();
extern void _GolgiNaR_reg();
extern void _GolgiSK2_reg();
extern void _Golgi_CALC_reg();
extern void _LeakConductance_reg();
extern void _MFGoCSyn_reg();
extern void _MFGoC_Syn_reg();
extern void _MF_Burst_reg();
extern void _MF_Input_reg();
extern void _MF_Poisson_reg();
extern void _stim_0_reg();
extern void _stim_1_reg();
extern void _stim_10_reg();
extern void _stim_11_reg();
extern void _stim_12_reg();
extern void _stim_13_reg();
extern void _stim_14_reg();
extern void _stim_15_reg();
extern void _stim_16_reg();
extern void _stim_17_reg();
extern void _stim_18_reg();
extern void _stim_19_reg();
extern void _stim_2_reg();
extern void _stim_20_reg();
extern void _stim_3_reg();
extern void _stim_4_reg();
extern void _stim_5_reg();
extern void _stim_6_reg();
extern void _stim_7_reg();
extern void _stim_8_reg();
extern void _stim_9_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," AlphaSyn.mod");
fprintf(stderr," GJ_0.mod");
fprintf(stderr," GapJuncCML.mod");
fprintf(stderr," GolgiBK.mod");
fprintf(stderr," GolgiCaHVA.mod");
fprintf(stderr," GolgiCaLVA.mod");
fprintf(stderr," GolgiHCN1f.mod");
fprintf(stderr," GolgiHCN1s.mod");
fprintf(stderr," GolgiHCN2f.mod");
fprintf(stderr," GolgiHCN2s.mod");
fprintf(stderr," GolgiKA.mod");
fprintf(stderr," GolgiKM.mod");
fprintf(stderr," GolgiKV.mod");
fprintf(stderr," GolgiNa.mod");
fprintf(stderr," GolgiNaP.mod");
fprintf(stderr," GolgiNaR.mod");
fprintf(stderr," GolgiSK2.mod");
fprintf(stderr," Golgi_CALC.mod");
fprintf(stderr," LeakConductance.mod");
fprintf(stderr," MFGoCSyn.mod");
fprintf(stderr," MFGoC_Syn.mod");
fprintf(stderr," MF_Burst.mod");
fprintf(stderr," MF_Input.mod");
fprintf(stderr," MF_Poisson.mod");
fprintf(stderr," stim_0.mod");
fprintf(stderr," stim_1.mod");
fprintf(stderr," stim_10.mod");
fprintf(stderr," stim_11.mod");
fprintf(stderr," stim_12.mod");
fprintf(stderr," stim_13.mod");
fprintf(stderr," stim_14.mod");
fprintf(stderr," stim_15.mod");
fprintf(stderr," stim_16.mod");
fprintf(stderr," stim_17.mod");
fprintf(stderr," stim_18.mod");
fprintf(stderr," stim_19.mod");
fprintf(stderr," stim_2.mod");
fprintf(stderr," stim_20.mod");
fprintf(stderr," stim_3.mod");
fprintf(stderr," stim_4.mod");
fprintf(stderr," stim_5.mod");
fprintf(stderr," stim_6.mod");
fprintf(stderr," stim_7.mod");
fprintf(stderr," stim_8.mod");
fprintf(stderr," stim_9.mod");
fprintf(stderr, "\n");
    }
_AlphaSyn_reg();
_GJ_0_reg();
_GapJuncCML_reg();
_GolgiBK_reg();
_GolgiCaHVA_reg();
_GolgiCaLVA_reg();
_GolgiHCN1f_reg();
_GolgiHCN1s_reg();
_GolgiHCN2f_reg();
_GolgiHCN2s_reg();
_GolgiKA_reg();
_GolgiKM_reg();
_GolgiKV_reg();
_GolgiNa_reg();
_GolgiNaP_reg();
_GolgiNaR_reg();
_GolgiSK2_reg();
_Golgi_CALC_reg();
_LeakConductance_reg();
_MFGoCSyn_reg();
_MFGoC_Syn_reg();
_MF_Burst_reg();
_MF_Input_reg();
_MF_Poisson_reg();
_stim_0_reg();
_stim_1_reg();
_stim_10_reg();
_stim_11_reg();
_stim_12_reg();
_stim_13_reg();
_stim_14_reg();
_stim_15_reg();
_stim_16_reg();
_stim_17_reg();
_stim_18_reg();
_stim_19_reg();
_stim_2_reg();
_stim_20_reg();
_stim_3_reg();
_stim_4_reg();
_stim_5_reg();
_stim_6_reg();
_stim_7_reg();
_stim_8_reg();
_stim_9_reg();
}
