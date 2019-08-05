/* Created by Language version: 7.5.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__GolgiSK2
#define _nrn_initial _nrn_initial__GolgiSK2
#define nrn_cur _nrn_cur__GolgiSK2
#define _nrn_current _nrn_current__GolgiSK2
#define nrn_jacob _nrn_jacob__GolgiSK2
#define nrn_state _nrn_state__GolgiSK2
#define _net_receive _net_receive__GolgiSK2 
#define activation activation__GolgiSK2 
#define rates rates__GolgiSK2 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gmax _p[0]
#define conductance _p[1]
#define n_instances _p[2]
#define n_q10Settings_q10Factor _p[3]
#define n_q10Settings_experimentalTemp _p[4]
#define n_q10Settings_TENDEGREES _p[5]
#define n_c1_relativeConductance _p[6]
#define n_c2_relativeConductance _p[7]
#define n_c3_relativeConductance _p[8]
#define n_c4_relativeConductance _p[9]
#define n_o1_relativeConductance _p[10]
#define n_o2_relativeConductance _p[11]
#define n_alpha_c1_c2_SEC _p[12]
#define n_alpha_c1_c2_rate_TIME_SCALE _p[13]
#define n_alpha_c1_c2_rate_VOLT_SCALE _p[14]
#define n_alpha_c1_c2_rate_CONC_SCALE _p[15]
#define n_alpha_c1_c2_rate_invc1 _p[16]
#define n_alpha_c1_c2_rate_invc2 _p[17]
#define n_alpha_c1_c2_rate_invc3 _p[18]
#define n_alpha_c1_c2_rate_invo1 _p[19]
#define n_alpha_c1_c2_rate_invo2 _p[20]
#define n_alpha_c1_c2_rate_diro1 _p[21]
#define n_alpha_c1_c2_rate_diro2 _p[22]
#define n_alpha_c1_c2_rate_dirc2 _p[23]
#define n_alpha_c1_c2_rate_dirc3 _p[24]
#define n_alpha_c1_c2_rate_dirc4 _p[25]
#define n_alpha_c1_c2_rate_diff _p[26]
#define n_beta_c2_c1_SEC _p[27]
#define n_beta_c2_c1_rate_TIME_SCALE _p[28]
#define n_beta_c2_c1_rate_VOLT_SCALE _p[29]
#define n_beta_c2_c1_rate_CONC_SCALE _p[30]
#define n_beta_c2_c1_rate_invc1 _p[31]
#define n_beta_c2_c1_rate_invc2 _p[32]
#define n_beta_c2_c1_rate_invc3 _p[33]
#define n_beta_c2_c1_rate_invo1 _p[34]
#define n_beta_c2_c1_rate_invo2 _p[35]
#define n_beta_c2_c1_rate_diro1 _p[36]
#define n_beta_c2_c1_rate_diro2 _p[37]
#define n_beta_c2_c1_rate_dirc2 _p[38]
#define n_beta_c2_c1_rate_dirc3 _p[39]
#define n_beta_c2_c1_rate_dirc4 _p[40]
#define n_beta_c2_c1_rate_diff _p[41]
#define n_alpha_c2_c3_SEC _p[42]
#define n_alpha_c2_c3_rate_TIME_SCALE _p[43]
#define n_alpha_c2_c3_rate_VOLT_SCALE _p[44]
#define n_alpha_c2_c3_rate_CONC_SCALE _p[45]
#define n_alpha_c2_c3_rate_invc1 _p[46]
#define n_alpha_c2_c3_rate_invc2 _p[47]
#define n_alpha_c2_c3_rate_invc3 _p[48]
#define n_alpha_c2_c3_rate_invo1 _p[49]
#define n_alpha_c2_c3_rate_invo2 _p[50]
#define n_alpha_c2_c3_rate_diro1 _p[51]
#define n_alpha_c2_c3_rate_diro2 _p[52]
#define n_alpha_c2_c3_rate_dirc2 _p[53]
#define n_alpha_c2_c3_rate_dirc3 _p[54]
#define n_alpha_c2_c3_rate_dirc4 _p[55]
#define n_alpha_c2_c3_rate_diff _p[56]
#define n_beta_c3_c2_SEC _p[57]
#define n_beta_c3_c2_rate_TIME_SCALE _p[58]
#define n_beta_c3_c2_rate_VOLT_SCALE _p[59]
#define n_beta_c3_c2_rate_CONC_SCALE _p[60]
#define n_beta_c3_c2_rate_invc1 _p[61]
#define n_beta_c3_c2_rate_invc2 _p[62]
#define n_beta_c3_c2_rate_invc3 _p[63]
#define n_beta_c3_c2_rate_invo1 _p[64]
#define n_beta_c3_c2_rate_invo2 _p[65]
#define n_beta_c3_c2_rate_diro1 _p[66]
#define n_beta_c3_c2_rate_diro2 _p[67]
#define n_beta_c3_c2_rate_dirc2 _p[68]
#define n_beta_c3_c2_rate_dirc3 _p[69]
#define n_beta_c3_c2_rate_dirc4 _p[70]
#define n_beta_c3_c2_rate_diff _p[71]
#define n_alpha_c3_c4_SEC _p[72]
#define n_alpha_c3_c4_rate_TIME_SCALE _p[73]
#define n_alpha_c3_c4_rate_VOLT_SCALE _p[74]
#define n_alpha_c3_c4_rate_CONC_SCALE _p[75]
#define n_alpha_c3_c4_rate_invc1 _p[76]
#define n_alpha_c3_c4_rate_invc2 _p[77]
#define n_alpha_c3_c4_rate_invc3 _p[78]
#define n_alpha_c3_c4_rate_invo1 _p[79]
#define n_alpha_c3_c4_rate_invo2 _p[80]
#define n_alpha_c3_c4_rate_diro1 _p[81]
#define n_alpha_c3_c4_rate_diro2 _p[82]
#define n_alpha_c3_c4_rate_dirc2 _p[83]
#define n_alpha_c3_c4_rate_dirc3 _p[84]
#define n_alpha_c3_c4_rate_dirc4 _p[85]
#define n_alpha_c3_c4_rate_diff _p[86]
#define n_beta_c4_c3_SEC _p[87]
#define n_beta_c4_c3_rate_TIME_SCALE _p[88]
#define n_beta_c4_c3_rate_VOLT_SCALE _p[89]
#define n_beta_c4_c3_rate_CONC_SCALE _p[90]
#define n_beta_c4_c3_rate_invc1 _p[91]
#define n_beta_c4_c3_rate_invc2 _p[92]
#define n_beta_c4_c3_rate_invc3 _p[93]
#define n_beta_c4_c3_rate_invo1 _p[94]
#define n_beta_c4_c3_rate_invo2 _p[95]
#define n_beta_c4_c3_rate_diro1 _p[96]
#define n_beta_c4_c3_rate_diro2 _p[97]
#define n_beta_c4_c3_rate_dirc2 _p[98]
#define n_beta_c4_c3_rate_dirc3 _p[99]
#define n_beta_c4_c3_rate_dirc4 _p[100]
#define n_beta_c4_c3_rate_diff _p[101]
#define n_alpha_c3_o1_SEC _p[102]
#define n_alpha_c3_o1_rate_TIME_SCALE _p[103]
#define n_alpha_c3_o1_rate_VOLT_SCALE _p[104]
#define n_alpha_c3_o1_rate_CONC_SCALE _p[105]
#define n_alpha_c3_o1_rate_invc1 _p[106]
#define n_alpha_c3_o1_rate_invc2 _p[107]
#define n_alpha_c3_o1_rate_invc3 _p[108]
#define n_alpha_c3_o1_rate_invo1 _p[109]
#define n_alpha_c3_o1_rate_invo2 _p[110]
#define n_alpha_c3_o1_rate_diro1 _p[111]
#define n_alpha_c3_o1_rate_diro2 _p[112]
#define n_alpha_c3_o1_rate_dirc2 _p[113]
#define n_alpha_c3_o1_rate_dirc3 _p[114]
#define n_alpha_c3_o1_rate_dirc4 _p[115]
#define n_alpha_c3_o1_rate_diff _p[116]
#define n_beta_o1_c3_SEC _p[117]
#define n_beta_o1_c3_rate_TIME_SCALE _p[118]
#define n_beta_o1_c3_rate_VOLT_SCALE _p[119]
#define n_beta_o1_c3_rate_CONC_SCALE _p[120]
#define n_beta_o1_c3_rate_invc1 _p[121]
#define n_beta_o1_c3_rate_invc2 _p[122]
#define n_beta_o1_c3_rate_invc3 _p[123]
#define n_beta_o1_c3_rate_invo1 _p[124]
#define n_beta_o1_c3_rate_invo2 _p[125]
#define n_beta_o1_c3_rate_diro1 _p[126]
#define n_beta_o1_c3_rate_diro2 _p[127]
#define n_beta_o1_c3_rate_dirc2 _p[128]
#define n_beta_o1_c3_rate_dirc3 _p[129]
#define n_beta_o1_c3_rate_dirc4 _p[130]
#define n_beta_o1_c3_rate_diff _p[131]
#define n_alpha_c4_o2_SEC _p[132]
#define n_alpha_c4_o2_rate_TIME_SCALE _p[133]
#define n_alpha_c4_o2_rate_VOLT_SCALE _p[134]
#define n_alpha_c4_o2_rate_CONC_SCALE _p[135]
#define n_alpha_c4_o2_rate_invc1 _p[136]
#define n_alpha_c4_o2_rate_invc2 _p[137]
#define n_alpha_c4_o2_rate_invc3 _p[138]
#define n_alpha_c4_o2_rate_invo1 _p[139]
#define n_alpha_c4_o2_rate_invo2 _p[140]
#define n_alpha_c4_o2_rate_diro1 _p[141]
#define n_alpha_c4_o2_rate_diro2 _p[142]
#define n_alpha_c4_o2_rate_dirc2 _p[143]
#define n_alpha_c4_o2_rate_dirc3 _p[144]
#define n_alpha_c4_o2_rate_dirc4 _p[145]
#define n_alpha_c4_o2_rate_diff _p[146]
#define n_beta_o2_c4_SEC _p[147]
#define n_beta_o2_c4_rate_TIME_SCALE _p[148]
#define n_beta_o2_c4_rate_VOLT_SCALE _p[149]
#define n_beta_o2_c4_rate_CONC_SCALE _p[150]
#define n_beta_o2_c4_rate_invc1 _p[151]
#define n_beta_o2_c4_rate_invc2 _p[152]
#define n_beta_o2_c4_rate_invc3 _p[153]
#define n_beta_o2_c4_rate_invo1 _p[154]
#define n_beta_o2_c4_rate_invo2 _p[155]
#define n_beta_o2_c4_rate_diro1 _p[156]
#define n_beta_o2_c4_rate_diro2 _p[157]
#define n_beta_o2_c4_rate_dirc2 _p[158]
#define n_beta_o2_c4_rate_dirc3 _p[159]
#define n_beta_o2_c4_rate_dirc4 _p[160]
#define n_beta_o2_c4_rate_diff _p[161]
#define gion _p[162]
#define n_q10Settings_q10 _p[163]
#define n_c1_q _p[164]
#define n_c2_q _p[165]
#define n_c3_q _p[166]
#define n_c4_q _p[167]
#define n_o1_q _p[168]
#define n_o2_q _p[169]
#define n_alpha_c1_c2_rate_V _p[170]
#define n_alpha_c1_c2_rate_ca_conc _p[171]
#define n_alpha_c1_c2_rate_inVc1 _p[172]
#define n_alpha_c1_c2_rate_inVc2 _p[173]
#define n_alpha_c1_c2_rate_inVc3 _p[174]
#define n_alpha_c1_c2_rate_inVo1 _p[175]
#define n_alpha_c1_c2_rate_inVo2 _p[176]
#define n_alpha_c1_c2_rate_r _p[177]
#define n_alpha_c1_c2_rf0 _p[178]
#define n_alpha_c1_c2_rf _p[179]
#define n_alpha_c1_c2_rr _p[180]
#define n_beta_c2_c1_rate_V _p[181]
#define n_beta_c2_c1_rate_ca_conc _p[182]
#define n_beta_c2_c1_rate_inVc1 _p[183]
#define n_beta_c2_c1_rate_inVc2 _p[184]
#define n_beta_c2_c1_rate_inVc3 _p[185]
#define n_beta_c2_c1_rate_inVo1 _p[186]
#define n_beta_c2_c1_rate_inVo2 _p[187]
#define n_beta_c2_c1_rate_r _p[188]
#define n_beta_c2_c1_rr0 _p[189]
#define n_beta_c2_c1_rr _p[190]
#define n_beta_c2_c1_rf _p[191]
#define n_alpha_c2_c3_rate_V _p[192]
#define n_alpha_c2_c3_rate_ca_conc _p[193]
#define n_alpha_c2_c3_rate_inVc1 _p[194]
#define n_alpha_c2_c3_rate_inVc2 _p[195]
#define n_alpha_c2_c3_rate_inVc3 _p[196]
#define n_alpha_c2_c3_rate_inVo1 _p[197]
#define n_alpha_c2_c3_rate_inVo2 _p[198]
#define n_alpha_c2_c3_rate_r _p[199]
#define n_alpha_c2_c3_rf0 _p[200]
#define n_alpha_c2_c3_rf _p[201]
#define n_alpha_c2_c3_rr _p[202]
#define n_beta_c3_c2_rate_V _p[203]
#define n_beta_c3_c2_rate_ca_conc _p[204]
#define n_beta_c3_c2_rate_inVc1 _p[205]
#define n_beta_c3_c2_rate_inVc2 _p[206]
#define n_beta_c3_c2_rate_inVc3 _p[207]
#define n_beta_c3_c2_rate_inVo1 _p[208]
#define n_beta_c3_c2_rate_inVo2 _p[209]
#define n_beta_c3_c2_rate_r _p[210]
#define n_beta_c3_c2_rr0 _p[211]
#define n_beta_c3_c2_rr _p[212]
#define n_beta_c3_c2_rf _p[213]
#define n_alpha_c3_c4_rate_V _p[214]
#define n_alpha_c3_c4_rate_ca_conc _p[215]
#define n_alpha_c3_c4_rate_inVc1 _p[216]
#define n_alpha_c3_c4_rate_inVc2 _p[217]
#define n_alpha_c3_c4_rate_inVc3 _p[218]
#define n_alpha_c3_c4_rate_inVo1 _p[219]
#define n_alpha_c3_c4_rate_inVo2 _p[220]
#define n_alpha_c3_c4_rate_r _p[221]
#define n_alpha_c3_c4_rf0 _p[222]
#define n_alpha_c3_c4_rf _p[223]
#define n_alpha_c3_c4_rr _p[224]
#define n_beta_c4_c3_rate_V _p[225]
#define n_beta_c4_c3_rate_ca_conc _p[226]
#define n_beta_c4_c3_rate_inVc1 _p[227]
#define n_beta_c4_c3_rate_inVc2 _p[228]
#define n_beta_c4_c3_rate_inVc3 _p[229]
#define n_beta_c4_c3_rate_inVo1 _p[230]
#define n_beta_c4_c3_rate_inVo2 _p[231]
#define n_beta_c4_c3_rate_r _p[232]
#define n_beta_c4_c3_rr0 _p[233]
#define n_beta_c4_c3_rr _p[234]
#define n_beta_c4_c3_rf _p[235]
#define n_alpha_c3_o1_rate_V _p[236]
#define n_alpha_c3_o1_rate_ca_conc _p[237]
#define n_alpha_c3_o1_rate_inVc1 _p[238]
#define n_alpha_c3_o1_rate_inVc2 _p[239]
#define n_alpha_c3_o1_rate_inVc3 _p[240]
#define n_alpha_c3_o1_rate_inVo1 _p[241]
#define n_alpha_c3_o1_rate_inVo2 _p[242]
#define n_alpha_c3_o1_rate_r _p[243]
#define n_alpha_c3_o1_rf0 _p[244]
#define n_alpha_c3_o1_rf _p[245]
#define n_alpha_c3_o1_rr _p[246]
#define n_beta_o1_c3_rate_V _p[247]
#define n_beta_o1_c3_rate_ca_conc _p[248]
#define n_beta_o1_c3_rate_inVc1 _p[249]
#define n_beta_o1_c3_rate_inVc2 _p[250]
#define n_beta_o1_c3_rate_inVc3 _p[251]
#define n_beta_o1_c3_rate_inVo1 _p[252]
#define n_beta_o1_c3_rate_inVo2 _p[253]
#define n_beta_o1_c3_rate_r _p[254]
#define n_beta_o1_c3_rr0 _p[255]
#define n_beta_o1_c3_rr _p[256]
#define n_beta_o1_c3_rf _p[257]
#define n_alpha_c4_o2_rate_V _p[258]
#define n_alpha_c4_o2_rate_ca_conc _p[259]
#define n_alpha_c4_o2_rate_inVc1 _p[260]
#define n_alpha_c4_o2_rate_inVc2 _p[261]
#define n_alpha_c4_o2_rate_inVc3 _p[262]
#define n_alpha_c4_o2_rate_inVo1 _p[263]
#define n_alpha_c4_o2_rate_inVo2 _p[264]
#define n_alpha_c4_o2_rate_r _p[265]
#define n_alpha_c4_o2_rf0 _p[266]
#define n_alpha_c4_o2_rf _p[267]
#define n_alpha_c4_o2_rr _p[268]
#define n_beta_o2_c4_rate_V _p[269]
#define n_beta_o2_c4_rate_ca_conc _p[270]
#define n_beta_o2_c4_rate_inVc1 _p[271]
#define n_beta_o2_c4_rate_inVc2 _p[272]
#define n_beta_o2_c4_rate_inVc3 _p[273]
#define n_beta_o2_c4_rate_inVo1 _p[274]
#define n_beta_o2_c4_rate_inVo2 _p[275]
#define n_beta_o2_c4_rate_r _p[276]
#define n_beta_o2_c4_rr0 _p[277]
#define n_beta_o2_c4_rr _p[278]
#define n_beta_o2_c4_rf _p[279]
#define n_rateScale _p[280]
#define n_q _p[281]
#define n_fcond _p[282]
#define fopen _p[283]
#define g _p[284]
#define n_c1_occupancy _p[285]
#define n_c2_occupancy _p[286]
#define n_c3_occupancy _p[287]
#define n_c4_occupancy _p[288]
#define n_o1_occupancy _p[289]
#define n_o2_occupancy _p[290]
#define temperature _p[291]
#define ek _p[292]
#define ik _p[293]
#define cai _p[294]
#define cao _p[295]
#define Dn_c1_occupancy _p[296]
#define Dn_c2_occupancy _p[297]
#define Dn_c3_occupancy _p[298]
#define Dn_c4_occupancy _p[299]
#define Dn_o1_occupancy _p[300]
#define Dn_o2_occupancy _p[301]
#define v _p[302]
#define _g _p[303]
#define _ion_cai	*_ppvar[0]._pval
#define _ion_cao	*_ppvar[1]._pval
#define _ion_ik	*_ppvar[2]._pval
#define _ion_dikdv	*_ppvar[3]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_rates(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_GolgiSK2", _hoc_setdata,
 "rates_GolgiSK2", _hoc_rates,
 0, 0
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "gmax_GolgiSK2", "S/cm2",
 "conductance_GolgiSK2", "uS",
 "n_q10Settings_experimentalTemp_GolgiSK2", "K",
 "n_q10Settings_TENDEGREES_GolgiSK2", "K",
 "n_alpha_c1_c2_SEC_GolgiSK2", "ms",
 "n_alpha_c1_c2_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_alpha_c1_c2_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_alpha_c1_c2_rate_CONC_SCALE_GolgiSK2", "mM",
 "n_beta_c2_c1_SEC_GolgiSK2", "ms",
 "n_beta_c2_c1_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_beta_c2_c1_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_beta_c2_c1_rate_CONC_SCALE_GolgiSK2", "mM",
 "n_alpha_c2_c3_SEC_GolgiSK2", "ms",
 "n_alpha_c2_c3_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_alpha_c2_c3_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_alpha_c2_c3_rate_CONC_SCALE_GolgiSK2", "mM",
 "n_beta_c3_c2_SEC_GolgiSK2", "ms",
 "n_beta_c3_c2_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_beta_c3_c2_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_beta_c3_c2_rate_CONC_SCALE_GolgiSK2", "mM",
 "n_alpha_c3_c4_SEC_GolgiSK2", "ms",
 "n_alpha_c3_c4_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_alpha_c3_c4_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_alpha_c3_c4_rate_CONC_SCALE_GolgiSK2", "mM",
 "n_beta_c4_c3_SEC_GolgiSK2", "ms",
 "n_beta_c4_c3_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_beta_c4_c3_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_beta_c4_c3_rate_CONC_SCALE_GolgiSK2", "mM",
 "n_alpha_c3_o1_SEC_GolgiSK2", "ms",
 "n_alpha_c3_o1_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_alpha_c3_o1_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_alpha_c3_o1_rate_CONC_SCALE_GolgiSK2", "mM",
 "n_beta_o1_c3_SEC_GolgiSK2", "ms",
 "n_beta_o1_c3_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_beta_o1_c3_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_beta_o1_c3_rate_CONC_SCALE_GolgiSK2", "mM",
 "n_alpha_c4_o2_SEC_GolgiSK2", "ms",
 "n_alpha_c4_o2_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_alpha_c4_o2_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_alpha_c4_o2_rate_CONC_SCALE_GolgiSK2", "mM",
 "n_beta_o2_c4_SEC_GolgiSK2", "ms",
 "n_beta_o2_c4_rate_TIME_SCALE_GolgiSK2", "ms",
 "n_beta_o2_c4_rate_VOLT_SCALE_GolgiSK2", "mV",
 "n_beta_o2_c4_rate_CONC_SCALE_GolgiSK2", "mM",
 "gion_GolgiSK2", "S/cm2",
 "n_alpha_c1_c2_rate_r_GolgiSK2", "kHz",
 "n_alpha_c1_c2_rf0_GolgiSK2", "kHz",
 "n_alpha_c1_c2_rf_GolgiSK2", "kHz",
 "n_alpha_c1_c2_rr_GolgiSK2", "kHz",
 "n_beta_c2_c1_rate_r_GolgiSK2", "kHz",
 "n_beta_c2_c1_rr0_GolgiSK2", "kHz",
 "n_beta_c2_c1_rr_GolgiSK2", "kHz",
 "n_beta_c2_c1_rf_GolgiSK2", "kHz",
 "n_alpha_c2_c3_rate_r_GolgiSK2", "kHz",
 "n_alpha_c2_c3_rf0_GolgiSK2", "kHz",
 "n_alpha_c2_c3_rf_GolgiSK2", "kHz",
 "n_alpha_c2_c3_rr_GolgiSK2", "kHz",
 "n_beta_c3_c2_rate_r_GolgiSK2", "kHz",
 "n_beta_c3_c2_rr0_GolgiSK2", "kHz",
 "n_beta_c3_c2_rr_GolgiSK2", "kHz",
 "n_beta_c3_c2_rf_GolgiSK2", "kHz",
 "n_alpha_c3_c4_rate_r_GolgiSK2", "kHz",
 "n_alpha_c3_c4_rf0_GolgiSK2", "kHz",
 "n_alpha_c3_c4_rf_GolgiSK2", "kHz",
 "n_alpha_c3_c4_rr_GolgiSK2", "kHz",
 "n_beta_c4_c3_rate_r_GolgiSK2", "kHz",
 "n_beta_c4_c3_rr0_GolgiSK2", "kHz",
 "n_beta_c4_c3_rr_GolgiSK2", "kHz",
 "n_beta_c4_c3_rf_GolgiSK2", "kHz",
 "n_alpha_c3_o1_rate_r_GolgiSK2", "kHz",
 "n_alpha_c3_o1_rf0_GolgiSK2", "kHz",
 "n_alpha_c3_o1_rf_GolgiSK2", "kHz",
 "n_alpha_c3_o1_rr_GolgiSK2", "kHz",
 "n_beta_o1_c3_rate_r_GolgiSK2", "kHz",
 "n_beta_o1_c3_rr0_GolgiSK2", "kHz",
 "n_beta_o1_c3_rr_GolgiSK2", "kHz",
 "n_beta_o1_c3_rf_GolgiSK2", "kHz",
 "n_alpha_c4_o2_rate_r_GolgiSK2", "kHz",
 "n_alpha_c4_o2_rf0_GolgiSK2", "kHz",
 "n_alpha_c4_o2_rf_GolgiSK2", "kHz",
 "n_alpha_c4_o2_rr_GolgiSK2", "kHz",
 "n_beta_o2_c4_rate_r_GolgiSK2", "kHz",
 "n_beta_o2_c4_rr0_GolgiSK2", "kHz",
 "n_beta_o2_c4_rr_GolgiSK2", "kHz",
 "n_beta_o2_c4_rf_GolgiSK2", "kHz",
 "g_GolgiSK2", "uS",
 0,0
};
 static double delta_t = 0.01;
 static double n_o2_occupancy0 = 0;
 static double n_o1_occupancy0 = 0;
 static double n_c4_occupancy0 = 0;
 static double n_c3_occupancy0 = 0;
 static double n_c2_occupancy0 = 0;
 static double n_c1_occupancy0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[4]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.5.0",
"GolgiSK2",
 "gmax_GolgiSK2",
 "conductance_GolgiSK2",
 "n_instances_GolgiSK2",
 "n_q10Settings_q10Factor_GolgiSK2",
 "n_q10Settings_experimentalTemp_GolgiSK2",
 "n_q10Settings_TENDEGREES_GolgiSK2",
 "n_c1_relativeConductance_GolgiSK2",
 "n_c2_relativeConductance_GolgiSK2",
 "n_c3_relativeConductance_GolgiSK2",
 "n_c4_relativeConductance_GolgiSK2",
 "n_o1_relativeConductance_GolgiSK2",
 "n_o2_relativeConductance_GolgiSK2",
 "n_alpha_c1_c2_SEC_GolgiSK2",
 "n_alpha_c1_c2_rate_TIME_SCALE_GolgiSK2",
 "n_alpha_c1_c2_rate_VOLT_SCALE_GolgiSK2",
 "n_alpha_c1_c2_rate_CONC_SCALE_GolgiSK2",
 "n_alpha_c1_c2_rate_invc1_GolgiSK2",
 "n_alpha_c1_c2_rate_invc2_GolgiSK2",
 "n_alpha_c1_c2_rate_invc3_GolgiSK2",
 "n_alpha_c1_c2_rate_invo1_GolgiSK2",
 "n_alpha_c1_c2_rate_invo2_GolgiSK2",
 "n_alpha_c1_c2_rate_diro1_GolgiSK2",
 "n_alpha_c1_c2_rate_diro2_GolgiSK2",
 "n_alpha_c1_c2_rate_dirc2_GolgiSK2",
 "n_alpha_c1_c2_rate_dirc3_GolgiSK2",
 "n_alpha_c1_c2_rate_dirc4_GolgiSK2",
 "n_alpha_c1_c2_rate_diff_GolgiSK2",
 "n_beta_c2_c1_SEC_GolgiSK2",
 "n_beta_c2_c1_rate_TIME_SCALE_GolgiSK2",
 "n_beta_c2_c1_rate_VOLT_SCALE_GolgiSK2",
 "n_beta_c2_c1_rate_CONC_SCALE_GolgiSK2",
 "n_beta_c2_c1_rate_invc1_GolgiSK2",
 "n_beta_c2_c1_rate_invc2_GolgiSK2",
 "n_beta_c2_c1_rate_invc3_GolgiSK2",
 "n_beta_c2_c1_rate_invo1_GolgiSK2",
 "n_beta_c2_c1_rate_invo2_GolgiSK2",
 "n_beta_c2_c1_rate_diro1_GolgiSK2",
 "n_beta_c2_c1_rate_diro2_GolgiSK2",
 "n_beta_c2_c1_rate_dirc2_GolgiSK2",
 "n_beta_c2_c1_rate_dirc3_GolgiSK2",
 "n_beta_c2_c1_rate_dirc4_GolgiSK2",
 "n_beta_c2_c1_rate_diff_GolgiSK2",
 "n_alpha_c2_c3_SEC_GolgiSK2",
 "n_alpha_c2_c3_rate_TIME_SCALE_GolgiSK2",
 "n_alpha_c2_c3_rate_VOLT_SCALE_GolgiSK2",
 "n_alpha_c2_c3_rate_CONC_SCALE_GolgiSK2",
 "n_alpha_c2_c3_rate_invc1_GolgiSK2",
 "n_alpha_c2_c3_rate_invc2_GolgiSK2",
 "n_alpha_c2_c3_rate_invc3_GolgiSK2",
 "n_alpha_c2_c3_rate_invo1_GolgiSK2",
 "n_alpha_c2_c3_rate_invo2_GolgiSK2",
 "n_alpha_c2_c3_rate_diro1_GolgiSK2",
 "n_alpha_c2_c3_rate_diro2_GolgiSK2",
 "n_alpha_c2_c3_rate_dirc2_GolgiSK2",
 "n_alpha_c2_c3_rate_dirc3_GolgiSK2",
 "n_alpha_c2_c3_rate_dirc4_GolgiSK2",
 "n_alpha_c2_c3_rate_diff_GolgiSK2",
 "n_beta_c3_c2_SEC_GolgiSK2",
 "n_beta_c3_c2_rate_TIME_SCALE_GolgiSK2",
 "n_beta_c3_c2_rate_VOLT_SCALE_GolgiSK2",
 "n_beta_c3_c2_rate_CONC_SCALE_GolgiSK2",
 "n_beta_c3_c2_rate_invc1_GolgiSK2",
 "n_beta_c3_c2_rate_invc2_GolgiSK2",
 "n_beta_c3_c2_rate_invc3_GolgiSK2",
 "n_beta_c3_c2_rate_invo1_GolgiSK2",
 "n_beta_c3_c2_rate_invo2_GolgiSK2",
 "n_beta_c3_c2_rate_diro1_GolgiSK2",
 "n_beta_c3_c2_rate_diro2_GolgiSK2",
 "n_beta_c3_c2_rate_dirc2_GolgiSK2",
 "n_beta_c3_c2_rate_dirc3_GolgiSK2",
 "n_beta_c3_c2_rate_dirc4_GolgiSK2",
 "n_beta_c3_c2_rate_diff_GolgiSK2",
 "n_alpha_c3_c4_SEC_GolgiSK2",
 "n_alpha_c3_c4_rate_TIME_SCALE_GolgiSK2",
 "n_alpha_c3_c4_rate_VOLT_SCALE_GolgiSK2",
 "n_alpha_c3_c4_rate_CONC_SCALE_GolgiSK2",
 "n_alpha_c3_c4_rate_invc1_GolgiSK2",
 "n_alpha_c3_c4_rate_invc2_GolgiSK2",
 "n_alpha_c3_c4_rate_invc3_GolgiSK2",
 "n_alpha_c3_c4_rate_invo1_GolgiSK2",
 "n_alpha_c3_c4_rate_invo2_GolgiSK2",
 "n_alpha_c3_c4_rate_diro1_GolgiSK2",
 "n_alpha_c3_c4_rate_diro2_GolgiSK2",
 "n_alpha_c3_c4_rate_dirc2_GolgiSK2",
 "n_alpha_c3_c4_rate_dirc3_GolgiSK2",
 "n_alpha_c3_c4_rate_dirc4_GolgiSK2",
 "n_alpha_c3_c4_rate_diff_GolgiSK2",
 "n_beta_c4_c3_SEC_GolgiSK2",
 "n_beta_c4_c3_rate_TIME_SCALE_GolgiSK2",
 "n_beta_c4_c3_rate_VOLT_SCALE_GolgiSK2",
 "n_beta_c4_c3_rate_CONC_SCALE_GolgiSK2",
 "n_beta_c4_c3_rate_invc1_GolgiSK2",
 "n_beta_c4_c3_rate_invc2_GolgiSK2",
 "n_beta_c4_c3_rate_invc3_GolgiSK2",
 "n_beta_c4_c3_rate_invo1_GolgiSK2",
 "n_beta_c4_c3_rate_invo2_GolgiSK2",
 "n_beta_c4_c3_rate_diro1_GolgiSK2",
 "n_beta_c4_c3_rate_diro2_GolgiSK2",
 "n_beta_c4_c3_rate_dirc2_GolgiSK2",
 "n_beta_c4_c3_rate_dirc3_GolgiSK2",
 "n_beta_c4_c3_rate_dirc4_GolgiSK2",
 "n_beta_c4_c3_rate_diff_GolgiSK2",
 "n_alpha_c3_o1_SEC_GolgiSK2",
 "n_alpha_c3_o1_rate_TIME_SCALE_GolgiSK2",
 "n_alpha_c3_o1_rate_VOLT_SCALE_GolgiSK2",
 "n_alpha_c3_o1_rate_CONC_SCALE_GolgiSK2",
 "n_alpha_c3_o1_rate_invc1_GolgiSK2",
 "n_alpha_c3_o1_rate_invc2_GolgiSK2",
 "n_alpha_c3_o1_rate_invc3_GolgiSK2",
 "n_alpha_c3_o1_rate_invo1_GolgiSK2",
 "n_alpha_c3_o1_rate_invo2_GolgiSK2",
 "n_alpha_c3_o1_rate_diro1_GolgiSK2",
 "n_alpha_c3_o1_rate_diro2_GolgiSK2",
 "n_alpha_c3_o1_rate_dirc2_GolgiSK2",
 "n_alpha_c3_o1_rate_dirc3_GolgiSK2",
 "n_alpha_c3_o1_rate_dirc4_GolgiSK2",
 "n_alpha_c3_o1_rate_diff_GolgiSK2",
 "n_beta_o1_c3_SEC_GolgiSK2",
 "n_beta_o1_c3_rate_TIME_SCALE_GolgiSK2",
 "n_beta_o1_c3_rate_VOLT_SCALE_GolgiSK2",
 "n_beta_o1_c3_rate_CONC_SCALE_GolgiSK2",
 "n_beta_o1_c3_rate_invc1_GolgiSK2",
 "n_beta_o1_c3_rate_invc2_GolgiSK2",
 "n_beta_o1_c3_rate_invc3_GolgiSK2",
 "n_beta_o1_c3_rate_invo1_GolgiSK2",
 "n_beta_o1_c3_rate_invo2_GolgiSK2",
 "n_beta_o1_c3_rate_diro1_GolgiSK2",
 "n_beta_o1_c3_rate_diro2_GolgiSK2",
 "n_beta_o1_c3_rate_dirc2_GolgiSK2",
 "n_beta_o1_c3_rate_dirc3_GolgiSK2",
 "n_beta_o1_c3_rate_dirc4_GolgiSK2",
 "n_beta_o1_c3_rate_diff_GolgiSK2",
 "n_alpha_c4_o2_SEC_GolgiSK2",
 "n_alpha_c4_o2_rate_TIME_SCALE_GolgiSK2",
 "n_alpha_c4_o2_rate_VOLT_SCALE_GolgiSK2",
 "n_alpha_c4_o2_rate_CONC_SCALE_GolgiSK2",
 "n_alpha_c4_o2_rate_invc1_GolgiSK2",
 "n_alpha_c4_o2_rate_invc2_GolgiSK2",
 "n_alpha_c4_o2_rate_invc3_GolgiSK2",
 "n_alpha_c4_o2_rate_invo1_GolgiSK2",
 "n_alpha_c4_o2_rate_invo2_GolgiSK2",
 "n_alpha_c4_o2_rate_diro1_GolgiSK2",
 "n_alpha_c4_o2_rate_diro2_GolgiSK2",
 "n_alpha_c4_o2_rate_dirc2_GolgiSK2",
 "n_alpha_c4_o2_rate_dirc3_GolgiSK2",
 "n_alpha_c4_o2_rate_dirc4_GolgiSK2",
 "n_alpha_c4_o2_rate_diff_GolgiSK2",
 "n_beta_o2_c4_SEC_GolgiSK2",
 "n_beta_o2_c4_rate_TIME_SCALE_GolgiSK2",
 "n_beta_o2_c4_rate_VOLT_SCALE_GolgiSK2",
 "n_beta_o2_c4_rate_CONC_SCALE_GolgiSK2",
 "n_beta_o2_c4_rate_invc1_GolgiSK2",
 "n_beta_o2_c4_rate_invc2_GolgiSK2",
 "n_beta_o2_c4_rate_invc3_GolgiSK2",
 "n_beta_o2_c4_rate_invo1_GolgiSK2",
 "n_beta_o2_c4_rate_invo2_GolgiSK2",
 "n_beta_o2_c4_rate_diro1_GolgiSK2",
 "n_beta_o2_c4_rate_diro2_GolgiSK2",
 "n_beta_o2_c4_rate_dirc2_GolgiSK2",
 "n_beta_o2_c4_rate_dirc3_GolgiSK2",
 "n_beta_o2_c4_rate_dirc4_GolgiSK2",
 "n_beta_o2_c4_rate_diff_GolgiSK2",
 0,
 "gion_GolgiSK2",
 "n_q10Settings_q10_GolgiSK2",
 "n_c1_q_GolgiSK2",
 "n_c2_q_GolgiSK2",
 "n_c3_q_GolgiSK2",
 "n_c4_q_GolgiSK2",
 "n_o1_q_GolgiSK2",
 "n_o2_q_GolgiSK2",
 "n_alpha_c1_c2_rate_V_GolgiSK2",
 "n_alpha_c1_c2_rate_ca_conc_GolgiSK2",
 "n_alpha_c1_c2_rate_inVc1_GolgiSK2",
 "n_alpha_c1_c2_rate_inVc2_GolgiSK2",
 "n_alpha_c1_c2_rate_inVc3_GolgiSK2",
 "n_alpha_c1_c2_rate_inVo1_GolgiSK2",
 "n_alpha_c1_c2_rate_inVo2_GolgiSK2",
 "n_alpha_c1_c2_rate_r_GolgiSK2",
 "n_alpha_c1_c2_rf0_GolgiSK2",
 "n_alpha_c1_c2_rf_GolgiSK2",
 "n_alpha_c1_c2_rr_GolgiSK2",
 "n_beta_c2_c1_rate_V_GolgiSK2",
 "n_beta_c2_c1_rate_ca_conc_GolgiSK2",
 "n_beta_c2_c1_rate_inVc1_GolgiSK2",
 "n_beta_c2_c1_rate_inVc2_GolgiSK2",
 "n_beta_c2_c1_rate_inVc3_GolgiSK2",
 "n_beta_c2_c1_rate_inVo1_GolgiSK2",
 "n_beta_c2_c1_rate_inVo2_GolgiSK2",
 "n_beta_c2_c1_rate_r_GolgiSK2",
 "n_beta_c2_c1_rr0_GolgiSK2",
 "n_beta_c2_c1_rr_GolgiSK2",
 "n_beta_c2_c1_rf_GolgiSK2",
 "n_alpha_c2_c3_rate_V_GolgiSK2",
 "n_alpha_c2_c3_rate_ca_conc_GolgiSK2",
 "n_alpha_c2_c3_rate_inVc1_GolgiSK2",
 "n_alpha_c2_c3_rate_inVc2_GolgiSK2",
 "n_alpha_c2_c3_rate_inVc3_GolgiSK2",
 "n_alpha_c2_c3_rate_inVo1_GolgiSK2",
 "n_alpha_c2_c3_rate_inVo2_GolgiSK2",
 "n_alpha_c2_c3_rate_r_GolgiSK2",
 "n_alpha_c2_c3_rf0_GolgiSK2",
 "n_alpha_c2_c3_rf_GolgiSK2",
 "n_alpha_c2_c3_rr_GolgiSK2",
 "n_beta_c3_c2_rate_V_GolgiSK2",
 "n_beta_c3_c2_rate_ca_conc_GolgiSK2",
 "n_beta_c3_c2_rate_inVc1_GolgiSK2",
 "n_beta_c3_c2_rate_inVc2_GolgiSK2",
 "n_beta_c3_c2_rate_inVc3_GolgiSK2",
 "n_beta_c3_c2_rate_inVo1_GolgiSK2",
 "n_beta_c3_c2_rate_inVo2_GolgiSK2",
 "n_beta_c3_c2_rate_r_GolgiSK2",
 "n_beta_c3_c2_rr0_GolgiSK2",
 "n_beta_c3_c2_rr_GolgiSK2",
 "n_beta_c3_c2_rf_GolgiSK2",
 "n_alpha_c3_c4_rate_V_GolgiSK2",
 "n_alpha_c3_c4_rate_ca_conc_GolgiSK2",
 "n_alpha_c3_c4_rate_inVc1_GolgiSK2",
 "n_alpha_c3_c4_rate_inVc2_GolgiSK2",
 "n_alpha_c3_c4_rate_inVc3_GolgiSK2",
 "n_alpha_c3_c4_rate_inVo1_GolgiSK2",
 "n_alpha_c3_c4_rate_inVo2_GolgiSK2",
 "n_alpha_c3_c4_rate_r_GolgiSK2",
 "n_alpha_c3_c4_rf0_GolgiSK2",
 "n_alpha_c3_c4_rf_GolgiSK2",
 "n_alpha_c3_c4_rr_GolgiSK2",
 "n_beta_c4_c3_rate_V_GolgiSK2",
 "n_beta_c4_c3_rate_ca_conc_GolgiSK2",
 "n_beta_c4_c3_rate_inVc1_GolgiSK2",
 "n_beta_c4_c3_rate_inVc2_GolgiSK2",
 "n_beta_c4_c3_rate_inVc3_GolgiSK2",
 "n_beta_c4_c3_rate_inVo1_GolgiSK2",
 "n_beta_c4_c3_rate_inVo2_GolgiSK2",
 "n_beta_c4_c3_rate_r_GolgiSK2",
 "n_beta_c4_c3_rr0_GolgiSK2",
 "n_beta_c4_c3_rr_GolgiSK2",
 "n_beta_c4_c3_rf_GolgiSK2",
 "n_alpha_c3_o1_rate_V_GolgiSK2",
 "n_alpha_c3_o1_rate_ca_conc_GolgiSK2",
 "n_alpha_c3_o1_rate_inVc1_GolgiSK2",
 "n_alpha_c3_o1_rate_inVc2_GolgiSK2",
 "n_alpha_c3_o1_rate_inVc3_GolgiSK2",
 "n_alpha_c3_o1_rate_inVo1_GolgiSK2",
 "n_alpha_c3_o1_rate_inVo2_GolgiSK2",
 "n_alpha_c3_o1_rate_r_GolgiSK2",
 "n_alpha_c3_o1_rf0_GolgiSK2",
 "n_alpha_c3_o1_rf_GolgiSK2",
 "n_alpha_c3_o1_rr_GolgiSK2",
 "n_beta_o1_c3_rate_V_GolgiSK2",
 "n_beta_o1_c3_rate_ca_conc_GolgiSK2",
 "n_beta_o1_c3_rate_inVc1_GolgiSK2",
 "n_beta_o1_c3_rate_inVc2_GolgiSK2",
 "n_beta_o1_c3_rate_inVc3_GolgiSK2",
 "n_beta_o1_c3_rate_inVo1_GolgiSK2",
 "n_beta_o1_c3_rate_inVo2_GolgiSK2",
 "n_beta_o1_c3_rate_r_GolgiSK2",
 "n_beta_o1_c3_rr0_GolgiSK2",
 "n_beta_o1_c3_rr_GolgiSK2",
 "n_beta_o1_c3_rf_GolgiSK2",
 "n_alpha_c4_o2_rate_V_GolgiSK2",
 "n_alpha_c4_o2_rate_ca_conc_GolgiSK2",
 "n_alpha_c4_o2_rate_inVc1_GolgiSK2",
 "n_alpha_c4_o2_rate_inVc2_GolgiSK2",
 "n_alpha_c4_o2_rate_inVc3_GolgiSK2",
 "n_alpha_c4_o2_rate_inVo1_GolgiSK2",
 "n_alpha_c4_o2_rate_inVo2_GolgiSK2",
 "n_alpha_c4_o2_rate_r_GolgiSK2",
 "n_alpha_c4_o2_rf0_GolgiSK2",
 "n_alpha_c4_o2_rf_GolgiSK2",
 "n_alpha_c4_o2_rr_GolgiSK2",
 "n_beta_o2_c4_rate_V_GolgiSK2",
 "n_beta_o2_c4_rate_ca_conc_GolgiSK2",
 "n_beta_o2_c4_rate_inVc1_GolgiSK2",
 "n_beta_o2_c4_rate_inVc2_GolgiSK2",
 "n_beta_o2_c4_rate_inVc3_GolgiSK2",
 "n_beta_o2_c4_rate_inVo1_GolgiSK2",
 "n_beta_o2_c4_rate_inVo2_GolgiSK2",
 "n_beta_o2_c4_rate_r_GolgiSK2",
 "n_beta_o2_c4_rr0_GolgiSK2",
 "n_beta_o2_c4_rr_GolgiSK2",
 "n_beta_o2_c4_rf_GolgiSK2",
 "n_rateScale_GolgiSK2",
 "n_q_GolgiSK2",
 "n_fcond_GolgiSK2",
 "fopen_GolgiSK2",
 "g_GolgiSK2",
 0,
 "n_c1_occupancy_GolgiSK2",
 "n_c2_occupancy_GolgiSK2",
 "n_c3_occupancy_GolgiSK2",
 "n_c4_occupancy_GolgiSK2",
 "n_o1_occupancy_GolgiSK2",
 "n_o2_occupancy_GolgiSK2",
 0,
 0};
 static Symbol* _ca_sym;
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 304, _prop);
 	/*initialize range parameters*/
 	gmax = 0;
 	conductance = 1e-005;
 	n_instances = 1;
 	n_q10Settings_q10Factor = 3;
 	n_q10Settings_experimentalTemp = 296.15;
 	n_q10Settings_TENDEGREES = 10;
 	n_c1_relativeConductance = 0;
 	n_c2_relativeConductance = 0;
 	n_c3_relativeConductance = 0;
 	n_c4_relativeConductance = 0;
 	n_o1_relativeConductance = 1;
 	n_o2_relativeConductance = 1;
 	n_alpha_c1_c2_SEC = 1000;
 	n_alpha_c1_c2_rate_TIME_SCALE = 1;
 	n_alpha_c1_c2_rate_VOLT_SCALE = 1;
 	n_alpha_c1_c2_rate_CONC_SCALE = 1e+006;
 	n_alpha_c1_c2_rate_invc1 = 0.08;
 	n_alpha_c1_c2_rate_invc2 = 0.08;
 	n_alpha_c1_c2_rate_invc3 = 0.2;
 	n_alpha_c1_c2_rate_invo1 = 1;
 	n_alpha_c1_c2_rate_invo2 = 0.1;
 	n_alpha_c1_c2_rate_diro1 = 0.16;
 	n_alpha_c1_c2_rate_diro2 = 1.2;
 	n_alpha_c1_c2_rate_dirc2 = 200;
 	n_alpha_c1_c2_rate_dirc3 = 160;
 	n_alpha_c1_c2_rate_dirc4 = 80;
 	n_alpha_c1_c2_rate_diff = 3;
 	n_beta_c2_c1_SEC = 1000;
 	n_beta_c2_c1_rate_TIME_SCALE = 1;
 	n_beta_c2_c1_rate_VOLT_SCALE = 1;
 	n_beta_c2_c1_rate_CONC_SCALE = 1e+006;
 	n_beta_c2_c1_rate_invc1 = 0.08;
 	n_beta_c2_c1_rate_invc2 = 0.08;
 	n_beta_c2_c1_rate_invc3 = 0.2;
 	n_beta_c2_c1_rate_invo1 = 1;
 	n_beta_c2_c1_rate_invo2 = 0.1;
 	n_beta_c2_c1_rate_diro1 = 0.16;
 	n_beta_c2_c1_rate_diro2 = 1.2;
 	n_beta_c2_c1_rate_dirc2 = 200;
 	n_beta_c2_c1_rate_dirc3 = 160;
 	n_beta_c2_c1_rate_dirc4 = 80;
 	n_beta_c2_c1_rate_diff = 3;
 	n_alpha_c2_c3_SEC = 1000;
 	n_alpha_c2_c3_rate_TIME_SCALE = 1;
 	n_alpha_c2_c3_rate_VOLT_SCALE = 1;
 	n_alpha_c2_c3_rate_CONC_SCALE = 1e+006;
 	n_alpha_c2_c3_rate_invc1 = 0.08;
 	n_alpha_c2_c3_rate_invc2 = 0.08;
 	n_alpha_c2_c3_rate_invc3 = 0.2;
 	n_alpha_c2_c3_rate_invo1 = 1;
 	n_alpha_c2_c3_rate_invo2 = 0.1;
 	n_alpha_c2_c3_rate_diro1 = 0.16;
 	n_alpha_c2_c3_rate_diro2 = 1.2;
 	n_alpha_c2_c3_rate_dirc2 = 200;
 	n_alpha_c2_c3_rate_dirc3 = 160;
 	n_alpha_c2_c3_rate_dirc4 = 80;
 	n_alpha_c2_c3_rate_diff = 3;
 	n_beta_c3_c2_SEC = 1000;
 	n_beta_c3_c2_rate_TIME_SCALE = 1;
 	n_beta_c3_c2_rate_VOLT_SCALE = 1;
 	n_beta_c3_c2_rate_CONC_SCALE = 1e+006;
 	n_beta_c3_c2_rate_invc1 = 0.08;
 	n_beta_c3_c2_rate_invc2 = 0.08;
 	n_beta_c3_c2_rate_invc3 = 0.2;
 	n_beta_c3_c2_rate_invo1 = 1;
 	n_beta_c3_c2_rate_invo2 = 0.1;
 	n_beta_c3_c2_rate_diro1 = 0.16;
 	n_beta_c3_c2_rate_diro2 = 1.2;
 	n_beta_c3_c2_rate_dirc2 = 200;
 	n_beta_c3_c2_rate_dirc3 = 160;
 	n_beta_c3_c2_rate_dirc4 = 80;
 	n_beta_c3_c2_rate_diff = 3;
 	n_alpha_c3_c4_SEC = 1000;
 	n_alpha_c3_c4_rate_TIME_SCALE = 1;
 	n_alpha_c3_c4_rate_VOLT_SCALE = 1;
 	n_alpha_c3_c4_rate_CONC_SCALE = 1e+006;
 	n_alpha_c3_c4_rate_invc1 = 0.08;
 	n_alpha_c3_c4_rate_invc2 = 0.08;
 	n_alpha_c3_c4_rate_invc3 = 0.2;
 	n_alpha_c3_c4_rate_invo1 = 1;
 	n_alpha_c3_c4_rate_invo2 = 0.1;
 	n_alpha_c3_c4_rate_diro1 = 0.16;
 	n_alpha_c3_c4_rate_diro2 = 1.2;
 	n_alpha_c3_c4_rate_dirc2 = 200;
 	n_alpha_c3_c4_rate_dirc3 = 160;
 	n_alpha_c3_c4_rate_dirc4 = 80;
 	n_alpha_c3_c4_rate_diff = 3;
 	n_beta_c4_c3_SEC = 1000;
 	n_beta_c4_c3_rate_TIME_SCALE = 1;
 	n_beta_c4_c3_rate_VOLT_SCALE = 1;
 	n_beta_c4_c3_rate_CONC_SCALE = 1e+006;
 	n_beta_c4_c3_rate_invc1 = 0.08;
 	n_beta_c4_c3_rate_invc2 = 0.08;
 	n_beta_c4_c3_rate_invc3 = 0.2;
 	n_beta_c4_c3_rate_invo1 = 1;
 	n_beta_c4_c3_rate_invo2 = 0.1;
 	n_beta_c4_c3_rate_diro1 = 0.16;
 	n_beta_c4_c3_rate_diro2 = 1.2;
 	n_beta_c4_c3_rate_dirc2 = 200;
 	n_beta_c4_c3_rate_dirc3 = 160;
 	n_beta_c4_c3_rate_dirc4 = 80;
 	n_beta_c4_c3_rate_diff = 3;
 	n_alpha_c3_o1_SEC = 1000;
 	n_alpha_c3_o1_rate_TIME_SCALE = 1;
 	n_alpha_c3_o1_rate_VOLT_SCALE = 1;
 	n_alpha_c3_o1_rate_CONC_SCALE = 1e+006;
 	n_alpha_c3_o1_rate_invc1 = 0.08;
 	n_alpha_c3_o1_rate_invc2 = 0.08;
 	n_alpha_c3_o1_rate_invc3 = 0.2;
 	n_alpha_c3_o1_rate_invo1 = 1;
 	n_alpha_c3_o1_rate_invo2 = 0.1;
 	n_alpha_c3_o1_rate_diro1 = 0.16;
 	n_alpha_c3_o1_rate_diro2 = 1.2;
 	n_alpha_c3_o1_rate_dirc2 = 200;
 	n_alpha_c3_o1_rate_dirc3 = 160;
 	n_alpha_c3_o1_rate_dirc4 = 80;
 	n_alpha_c3_o1_rate_diff = 3;
 	n_beta_o1_c3_SEC = 1000;
 	n_beta_o1_c3_rate_TIME_SCALE = 1;
 	n_beta_o1_c3_rate_VOLT_SCALE = 1;
 	n_beta_o1_c3_rate_CONC_SCALE = 1e+006;
 	n_beta_o1_c3_rate_invc1 = 0.08;
 	n_beta_o1_c3_rate_invc2 = 0.08;
 	n_beta_o1_c3_rate_invc3 = 0.2;
 	n_beta_o1_c3_rate_invo1 = 1;
 	n_beta_o1_c3_rate_invo2 = 0.1;
 	n_beta_o1_c3_rate_diro1 = 0.16;
 	n_beta_o1_c3_rate_diro2 = 1.2;
 	n_beta_o1_c3_rate_dirc2 = 200;
 	n_beta_o1_c3_rate_dirc3 = 160;
 	n_beta_o1_c3_rate_dirc4 = 80;
 	n_beta_o1_c3_rate_diff = 3;
 	n_alpha_c4_o2_SEC = 1000;
 	n_alpha_c4_o2_rate_TIME_SCALE = 1;
 	n_alpha_c4_o2_rate_VOLT_SCALE = 1;
 	n_alpha_c4_o2_rate_CONC_SCALE = 1e+006;
 	n_alpha_c4_o2_rate_invc1 = 0.08;
 	n_alpha_c4_o2_rate_invc2 = 0.08;
 	n_alpha_c4_o2_rate_invc3 = 0.2;
 	n_alpha_c4_o2_rate_invo1 = 1;
 	n_alpha_c4_o2_rate_invo2 = 0.1;
 	n_alpha_c4_o2_rate_diro1 = 0.16;
 	n_alpha_c4_o2_rate_diro2 = 1.2;
 	n_alpha_c4_o2_rate_dirc2 = 200;
 	n_alpha_c4_o2_rate_dirc3 = 160;
 	n_alpha_c4_o2_rate_dirc4 = 80;
 	n_alpha_c4_o2_rate_diff = 3;
 	n_beta_o2_c4_SEC = 1000;
 	n_beta_o2_c4_rate_TIME_SCALE = 1;
 	n_beta_o2_c4_rate_VOLT_SCALE = 1;
 	n_beta_o2_c4_rate_CONC_SCALE = 1e+006;
 	n_beta_o2_c4_rate_invc1 = 0.08;
 	n_beta_o2_c4_rate_invc2 = 0.08;
 	n_beta_o2_c4_rate_invc3 = 0.2;
 	n_beta_o2_c4_rate_invo1 = 1;
 	n_beta_o2_c4_rate_invo2 = 0.1;
 	n_beta_o2_c4_rate_diro1 = 0.16;
 	n_beta_o2_c4_rate_diro2 = 1.2;
 	n_beta_o2_c4_rate_dirc2 = 200;
 	n_beta_o2_c4_rate_dirc3 = 160;
 	n_beta_o2_c4_rate_dirc4 = 80;
 	n_beta_o2_c4_rate_diff = 3;
 	_prop->param = _p;
 	_prop->param_size = 304;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[0]._pval = &prop_ion->param[1]; /* cai */
 	_ppvar[1]._pval = &prop_ion->param[2]; /* cao */
 prop_ion = need_memb(_k_sym);
 	_ppvar[2]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[3]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _thread_cleanup(Datum*);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _GolgiSK2_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ca", 2.0);
 	ion_reg("k", 1.0);
 	_ca_sym = hoc_lookup("ca_ion");
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 3);
  _extcall_thread = (Datum*)ecalloc(2, sizeof(Datum));
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 304, 5);
  hoc_register_dparam_semantics(_mechtype, 0, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 GolgiSK2 D:/Work/Comp Models/Learn Neuroml2/nml2_goc/Mechanisms/GolgiSK2.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "Mod file for component: Component(id=GolgiSK2 type=ionChannelKS)";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_threadargsproto_);
 
#define _MATELM1(_row,_col) *(_nrn_thread_getelm(_so, _row + 1, _col + 1))
 
#define _RHS1(_arg) _rhs[_arg+1]
  static int _cvspth1 = 1;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 extern double *_nrn_thread_getelm();
 
#define _MATELM1(_row,_col) *(_nrn_thread_getelm(_so, _row + 1, _col + 1))
 
#define _RHS1(_arg) _rhs[_arg+1]
  
#define _linmat1  1
 static int _spth1 = 0;
 static int _slist1[6], _dlist1[6]; static double *_temp1;
 static int activation();
 
static int activation (void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt)
 {int _reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<6;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]] - _p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargs_ ) ;
   /* ~ n_c1_occupancy <-> n_c2_occupancy ( n_alpha_c1_c2_rate_r , n_beta_c2_c1_rate_r )*/
 f_flux =  n_alpha_c1_c2_rate_r * n_c1_occupancy ;
 b_flux =  n_beta_c2_c1_rate_r * n_c2_occupancy ;
 _RHS1( 5) -= (f_flux - b_flux);
 
 _term =  n_alpha_c1_c2_rate_r ;
 _MATELM1( 5 ,5)  += _term;
 _term =  n_beta_c2_c1_rate_r ;
 _MATELM1( 5 ,0)  -= _term;
 /*REACTION*/
  /* ~ n_c2_occupancy <-> n_c3_occupancy ( n_alpha_c2_c3_rate_r , n_beta_c3_c2_rate_r )*/
 f_flux =  n_alpha_c2_c3_rate_r * n_c2_occupancy ;
 b_flux =  n_beta_c3_c2_rate_r * n_c3_occupancy ;
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  n_alpha_c2_c3_rate_r ;
 _MATELM1( 4 ,0)  -= _term;
 _term =  n_beta_c3_c2_rate_r ;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ n_c3_occupancy <-> n_c4_occupancy ( n_alpha_c3_c4_rate_r , n_beta_c4_c3_rate_r )*/
 f_flux =  n_alpha_c3_c4_rate_r * n_c3_occupancy ;
 b_flux =  n_beta_c4_c3_rate_r * n_c4_occupancy ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  n_alpha_c3_c4_rate_r ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  n_beta_c4_c3_rate_r ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ n_c3_occupancy <-> n_o1_occupancy ( n_alpha_c3_o1_rate_r , n_beta_o1_c3_rate_r )*/
 f_flux =  n_alpha_c3_o1_rate_r * n_c3_occupancy ;
 b_flux =  n_beta_o1_c3_rate_r * n_o1_occupancy ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  n_alpha_c3_o1_rate_r ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 2 ,4)  -= _term;
 _term =  n_beta_o1_c3_rate_r ;
 _MATELM1( 4 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ n_c4_occupancy <-> n_o2_occupancy ( n_alpha_c4_o2_rate_r , n_beta_o2_c4_rate_r )*/
 f_flux =  n_alpha_c4_o2_rate_r * n_c4_occupancy ;
 b_flux =  n_beta_o2_c4_rate_r * n_o2_occupancy ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  n_alpha_c4_o2_rate_r ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 1 ,3)  -= _term;
 _term =  n_beta_o2_c4_rate_r ;
 _MATELM1( 3 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
   /* n_c1_occupancy + n_o1_occupancy + n_o2_occupancy + n_c3_occupancy + n_c4_occupancy + n_c2_occupancy = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= n_c2_occupancy ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= n_c4_occupancy ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= n_c3_occupancy ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= n_o2_occupancy ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= n_o1_occupancy ;
 _MATELM1(0, 5) = 1;
 _RHS1(0) -= n_c1_occupancy ;
 /*CONSERVATION*/
   } return _reset;
 }
 
static int  rates ( _threadargsproto_ ) {
   double _lcaConc ;
 _lcaConc = cai ;
   n_q10Settings_q10 = pow( n_q10Settings_q10Factor , ( ( temperature - n_q10Settings_experimentalTemp ) / n_q10Settings_TENDEGREES ) ) ;
   n_c1_q = n_c1_relativeConductance * n_c1_occupancy ;
   n_c2_q = n_c2_relativeConductance * n_c2_occupancy ;
   n_c3_q = n_c3_relativeConductance * n_c3_occupancy ;
   n_c4_q = n_c4_relativeConductance * n_c4_occupancy ;
   n_o1_q = n_o1_relativeConductance * n_o1_occupancy ;
   n_o2_q = n_o2_relativeConductance * n_o2_occupancy ;
   n_alpha_c1_c2_rate_V = v / n_alpha_c1_c2_rate_VOLT_SCALE ;
   n_alpha_c1_c2_rate_ca_conc = _lcaConc / n_alpha_c1_c2_rate_CONC_SCALE ;
   n_alpha_c1_c2_rate_inVc1 = n_alpha_c1_c2_rate_invc1 ;
   n_alpha_c1_c2_rate_inVc2 = n_alpha_c1_c2_rate_invc2 ;
   n_alpha_c1_c2_rate_inVc3 = n_alpha_c1_c2_rate_invc3 ;
   n_alpha_c1_c2_rate_inVo1 = n_alpha_c1_c2_rate_invo1 ;
   n_alpha_c1_c2_rate_inVo2 = n_alpha_c1_c2_rate_invo2 ;
   n_alpha_c1_c2_rate_r = n_rateScale * ( n_alpha_c1_c2_rate_dirc2 * ( n_alpha_c1_c2_rate_ca_conc * 1e6 / n_alpha_c1_c2_rate_diff ) ) / n_alpha_c1_c2_rate_TIME_SCALE ;
   n_alpha_c1_c2_rf0 = n_alpha_c1_c2_rate_r ;
   n_alpha_c1_c2_rf = n_alpha_c1_c2_rf0 ;
   n_alpha_c1_c2_rr = 0.0 / n_alpha_c1_c2_SEC ;
   n_beta_c2_c1_rate_V = v / n_beta_c2_c1_rate_VOLT_SCALE ;
   n_beta_c2_c1_rate_ca_conc = _lcaConc / n_beta_c2_c1_rate_CONC_SCALE ;
   n_beta_c2_c1_rate_inVc1 = n_beta_c2_c1_rate_invc1 ;
   n_beta_c2_c1_rate_inVc2 = n_beta_c2_c1_rate_invc2 ;
   n_beta_c2_c1_rate_inVc3 = n_beta_c2_c1_rate_invc3 ;
   n_beta_c2_c1_rate_inVo1 = n_beta_c2_c1_rate_invo1 ;
   n_beta_c2_c1_rate_inVo2 = n_beta_c2_c1_rate_invo2 ;
   n_beta_c2_c1_rate_r = n_rateScale * ( n_beta_c2_c1_rate_inVc1 ) / n_beta_c2_c1_rate_TIME_SCALE ;
   n_beta_c2_c1_rr0 = n_beta_c2_c1_rate_r ;
   n_beta_c2_c1_rr = n_beta_c2_c1_rr0 ;
   n_beta_c2_c1_rf = 0.0 / n_beta_c2_c1_SEC ;
   n_alpha_c2_c3_rate_V = v / n_alpha_c2_c3_rate_VOLT_SCALE ;
   n_alpha_c2_c3_rate_ca_conc = _lcaConc / n_alpha_c2_c3_rate_CONC_SCALE ;
   n_alpha_c2_c3_rate_inVc1 = n_alpha_c2_c3_rate_invc1 ;
   n_alpha_c2_c3_rate_inVc2 = n_alpha_c2_c3_rate_invc2 ;
   n_alpha_c2_c3_rate_inVc3 = n_alpha_c2_c3_rate_invc3 ;
   n_alpha_c2_c3_rate_inVo1 = n_alpha_c2_c3_rate_invo1 ;
   n_alpha_c2_c3_rate_inVo2 = n_alpha_c2_c3_rate_invo2 ;
   n_alpha_c2_c3_rate_r = n_rateScale * ( n_alpha_c2_c3_rate_dirc3 * ( n_alpha_c2_c3_rate_ca_conc * 1e6 / n_alpha_c2_c3_rate_diff ) ) / n_alpha_c2_c3_rate_TIME_SCALE ;
   n_alpha_c2_c3_rf0 = n_alpha_c2_c3_rate_r ;
   n_alpha_c2_c3_rf = n_alpha_c2_c3_rf0 ;
   n_alpha_c2_c3_rr = 0.0 / n_alpha_c2_c3_SEC ;
   n_beta_c3_c2_rate_V = v / n_beta_c3_c2_rate_VOLT_SCALE ;
   n_beta_c3_c2_rate_ca_conc = _lcaConc / n_beta_c3_c2_rate_CONC_SCALE ;
   n_beta_c3_c2_rate_inVc1 = n_beta_c3_c2_rate_invc1 ;
   n_beta_c3_c2_rate_inVc2 = n_beta_c3_c2_rate_invc2 ;
   n_beta_c3_c2_rate_inVc3 = n_beta_c3_c2_rate_invc3 ;
   n_beta_c3_c2_rate_inVo1 = n_beta_c3_c2_rate_invo1 ;
   n_beta_c3_c2_rate_inVo2 = n_beta_c3_c2_rate_invo2 ;
   n_beta_c3_c2_rate_r = n_rateScale * ( n_beta_c3_c2_rate_inVc2 ) / n_beta_c3_c2_rate_TIME_SCALE ;
   n_beta_c3_c2_rr0 = n_beta_c3_c2_rate_r ;
   n_beta_c3_c2_rr = n_beta_c3_c2_rr0 ;
   n_beta_c3_c2_rf = 0.0 / n_beta_c3_c2_SEC ;
   n_alpha_c3_c4_rate_V = v / n_alpha_c3_c4_rate_VOLT_SCALE ;
   n_alpha_c3_c4_rate_ca_conc = _lcaConc / n_alpha_c3_c4_rate_CONC_SCALE ;
   n_alpha_c3_c4_rate_inVc1 = n_alpha_c3_c4_rate_invc1 ;
   n_alpha_c3_c4_rate_inVc2 = n_alpha_c3_c4_rate_invc2 ;
   n_alpha_c3_c4_rate_inVc3 = n_alpha_c3_c4_rate_invc3 ;
   n_alpha_c3_c4_rate_inVo1 = n_alpha_c3_c4_rate_invo1 ;
   n_alpha_c3_c4_rate_inVo2 = n_alpha_c3_c4_rate_invo2 ;
   n_alpha_c3_c4_rate_r = n_rateScale * ( n_alpha_c3_c4_rate_dirc4 * ( n_alpha_c3_c4_rate_ca_conc * 1e6 / n_alpha_c3_c4_rate_diff ) ) / n_alpha_c3_c4_rate_TIME_SCALE ;
   n_alpha_c3_c4_rf0 = n_alpha_c3_c4_rate_r ;
   n_alpha_c3_c4_rf = n_alpha_c3_c4_rf0 ;
   n_alpha_c3_c4_rr = 0.0 / n_alpha_c3_c4_SEC ;
   n_beta_c4_c3_rate_V = v / n_beta_c4_c3_rate_VOLT_SCALE ;
   n_beta_c4_c3_rate_ca_conc = _lcaConc / n_beta_c4_c3_rate_CONC_SCALE ;
   n_beta_c4_c3_rate_inVc1 = n_beta_c4_c3_rate_invc1 ;
   n_beta_c4_c3_rate_inVc2 = n_beta_c4_c3_rate_invc2 ;
   n_beta_c4_c3_rate_inVc3 = n_beta_c4_c3_rate_invc3 ;
   n_beta_c4_c3_rate_inVo1 = n_beta_c4_c3_rate_invo1 ;
   n_beta_c4_c3_rate_inVo2 = n_beta_c4_c3_rate_invo2 ;
   n_beta_c4_c3_rate_r = n_rateScale * ( n_beta_c4_c3_rate_inVc3 ) / n_beta_c4_c3_rate_TIME_SCALE ;
   n_beta_c4_c3_rr0 = n_beta_c4_c3_rate_r ;
   n_beta_c4_c3_rr = n_beta_c4_c3_rr0 ;
   n_beta_c4_c3_rf = 0.0 / n_beta_c4_c3_SEC ;
   n_alpha_c3_o1_rate_V = v / n_alpha_c3_o1_rate_VOLT_SCALE ;
   n_alpha_c3_o1_rate_ca_conc = _lcaConc / n_alpha_c3_o1_rate_CONC_SCALE ;
   n_alpha_c3_o1_rate_inVc1 = n_alpha_c3_o1_rate_invc1 ;
   n_alpha_c3_o1_rate_inVc2 = n_alpha_c3_o1_rate_invc2 ;
   n_alpha_c3_o1_rate_inVc3 = n_alpha_c3_o1_rate_invc3 ;
   n_alpha_c3_o1_rate_inVo1 = n_alpha_c3_o1_rate_invo1 ;
   n_alpha_c3_o1_rate_inVo2 = n_alpha_c3_o1_rate_invo2 ;
   n_alpha_c3_o1_rate_r = n_rateScale * ( n_alpha_c3_o1_rate_diro1 ) / n_alpha_c3_o1_rate_TIME_SCALE ;
   n_alpha_c3_o1_rf0 = n_alpha_c3_o1_rate_r ;
   n_alpha_c3_o1_rf = n_alpha_c3_o1_rf0 ;
   n_alpha_c3_o1_rr = 0.0 / n_alpha_c3_o1_SEC ;
   n_beta_o1_c3_rate_V = v / n_beta_o1_c3_rate_VOLT_SCALE ;
   n_beta_o1_c3_rate_ca_conc = _lcaConc / n_beta_o1_c3_rate_CONC_SCALE ;
   n_beta_o1_c3_rate_inVc1 = n_beta_o1_c3_rate_invc1 ;
   n_beta_o1_c3_rate_inVc2 = n_beta_o1_c3_rate_invc2 ;
   n_beta_o1_c3_rate_inVc3 = n_beta_o1_c3_rate_invc3 ;
   n_beta_o1_c3_rate_inVo1 = n_beta_o1_c3_rate_invo1 ;
   n_beta_o1_c3_rate_inVo2 = n_beta_o1_c3_rate_invo2 ;
   n_beta_o1_c3_rate_r = n_rateScale * ( n_beta_o1_c3_rate_inVo1 ) / n_beta_o1_c3_rate_TIME_SCALE ;
   n_beta_o1_c3_rr0 = n_beta_o1_c3_rate_r ;
   n_beta_o1_c3_rr = n_beta_o1_c3_rr0 ;
   n_beta_o1_c3_rf = 0.0 / n_beta_o1_c3_SEC ;
   n_alpha_c4_o2_rate_V = v / n_alpha_c4_o2_rate_VOLT_SCALE ;
   n_alpha_c4_o2_rate_ca_conc = _lcaConc / n_alpha_c4_o2_rate_CONC_SCALE ;
   n_alpha_c4_o2_rate_inVc1 = n_alpha_c4_o2_rate_invc1 ;
   n_alpha_c4_o2_rate_inVc2 = n_alpha_c4_o2_rate_invc2 ;
   n_alpha_c4_o2_rate_inVc3 = n_alpha_c4_o2_rate_invc3 ;
   n_alpha_c4_o2_rate_inVo1 = n_alpha_c4_o2_rate_invo1 ;
   n_alpha_c4_o2_rate_inVo2 = n_alpha_c4_o2_rate_invo2 ;
   n_alpha_c4_o2_rate_r = n_rateScale * ( n_alpha_c4_o2_rate_diro2 ) / n_alpha_c4_o2_rate_TIME_SCALE ;
   n_alpha_c4_o2_rf0 = n_alpha_c4_o2_rate_r ;
   n_alpha_c4_o2_rf = n_alpha_c4_o2_rf0 ;
   n_alpha_c4_o2_rr = 0.0 / n_alpha_c4_o2_SEC ;
   n_beta_o2_c4_rate_V = v / n_beta_o2_c4_rate_VOLT_SCALE ;
   n_beta_o2_c4_rate_ca_conc = _lcaConc / n_beta_o2_c4_rate_CONC_SCALE ;
   n_beta_o2_c4_rate_inVc1 = n_beta_o2_c4_rate_invc1 ;
   n_beta_o2_c4_rate_inVc2 = n_beta_o2_c4_rate_invc2 ;
   n_beta_o2_c4_rate_inVc3 = n_beta_o2_c4_rate_invc3 ;
   n_beta_o2_c4_rate_inVo1 = n_beta_o2_c4_rate_invo1 ;
   n_beta_o2_c4_rate_inVo2 = n_beta_o2_c4_rate_invo2 ;
   n_beta_o2_c4_rate_r = n_rateScale * ( n_beta_o2_c4_rate_inVo2 ) / n_beta_o2_c4_rate_TIME_SCALE ;
   n_beta_o2_c4_rr0 = n_beta_o2_c4_rate_r ;
   n_beta_o2_c4_rr = n_beta_o2_c4_rr0 ;
   n_beta_o2_c4_rf = 0.0 / n_beta_o2_c4_SEC ;
   n_rateScale = n_q10Settings_q10 ;
   n_q = n_c1_q + n_c2_q + n_c3_q + n_c4_q + n_o1_q + n_o2_q ;
   n_fcond = pow( n_q , n_instances ) ;
    return 0; }
 
static void _hoc_rates(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 rates ( _p, _ppvar, _thread, _nt );
 hoc_retpushx(_r);
}
 
/*CVODE ode begin*/
 static int _ode_spec1(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<6;_i++) _p[_dlist1[_i]] = 0.0;}
 rates ( _threadargs_ ) ;
 /* ~ n_c1_occupancy <-> n_c2_occupancy ( n_alpha_c1_c2_rate_r , n_beta_c2_c1_rate_r )*/
 f_flux =  n_alpha_c1_c2_rate_r * n_c1_occupancy ;
 b_flux =  n_beta_c2_c1_rate_r * n_c2_occupancy ;
 Dn_c1_occupancy -= (f_flux - b_flux);
 Dn_c2_occupancy += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ n_c2_occupancy <-> n_c3_occupancy ( n_alpha_c2_c3_rate_r , n_beta_c3_c2_rate_r )*/
 f_flux =  n_alpha_c2_c3_rate_r * n_c2_occupancy ;
 b_flux =  n_beta_c3_c2_rate_r * n_c3_occupancy ;
 Dn_c2_occupancy -= (f_flux - b_flux);
 Dn_c3_occupancy += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ n_c3_occupancy <-> n_c4_occupancy ( n_alpha_c3_c4_rate_r , n_beta_c4_c3_rate_r )*/
 f_flux =  n_alpha_c3_c4_rate_r * n_c3_occupancy ;
 b_flux =  n_beta_c4_c3_rate_r * n_c4_occupancy ;
 Dn_c3_occupancy -= (f_flux - b_flux);
 Dn_c4_occupancy += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ n_c3_occupancy <-> n_o1_occupancy ( n_alpha_c3_o1_rate_r , n_beta_o1_c3_rate_r )*/
 f_flux =  n_alpha_c3_o1_rate_r * n_c3_occupancy ;
 b_flux =  n_beta_o1_c3_rate_r * n_o1_occupancy ;
 Dn_c3_occupancy -= (f_flux - b_flux);
 Dn_o1_occupancy += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ n_c4_occupancy <-> n_o2_occupancy ( n_alpha_c4_o2_rate_r , n_beta_o2_c4_rate_r )*/
 f_flux =  n_alpha_c4_o2_rate_r * n_c4_occupancy ;
 b_flux =  n_beta_o2_c4_rate_r * n_o2_occupancy ;
 Dn_c4_occupancy -= (f_flux - b_flux);
 Dn_o2_occupancy += (f_flux - b_flux);
 
 /*REACTION*/
   /* n_c1_occupancy + n_o1_occupancy + n_o2_occupancy + n_c3_occupancy + n_c4_occupancy + n_c2_occupancy = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<6;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargs_ ) ;
 /* ~ n_c1_occupancy <-> n_c2_occupancy ( n_alpha_c1_c2_rate_r , n_beta_c2_c1_rate_r )*/
 _term =  n_alpha_c1_c2_rate_r ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 0 ,5)  -= _term;
 _term =  n_beta_c2_c1_rate_r ;
 _MATELM1( 5 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ n_c2_occupancy <-> n_c3_occupancy ( n_alpha_c2_c3_rate_r , n_beta_c3_c2_rate_r )*/
 _term =  n_alpha_c2_c3_rate_r ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 4 ,0)  -= _term;
 _term =  n_beta_c3_c2_rate_r ;
 _MATELM1( 0 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ n_c3_occupancy <-> n_c4_occupancy ( n_alpha_c3_c4_rate_r , n_beta_c4_c3_rate_r )*/
 _term =  n_alpha_c3_c4_rate_r ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  n_beta_c4_c3_rate_r ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ n_c3_occupancy <-> n_o1_occupancy ( n_alpha_c3_o1_rate_r , n_beta_o1_c3_rate_r )*/
 _term =  n_alpha_c3_o1_rate_r ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 2 ,4)  -= _term;
 _term =  n_beta_o1_c3_rate_r ;
 _MATELM1( 4 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ n_c4_occupancy <-> n_o2_occupancy ( n_alpha_c4_o2_rate_r , n_beta_o2_c4_rate_r )*/
 _term =  n_alpha_c4_o2_rate_r ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 1 ,3)  -= _term;
 _term =  n_beta_o2_c4_rate_r ;
 _MATELM1( 3 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
   /* n_c1_occupancy + n_o1_occupancy + n_o2_occupancy + n_c3_occupancy + n_c4_occupancy + n_c2_occupancy = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 6;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
  cao = _ion_cao;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 6; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _cvode_sparse_thread(&_thread[_cvspth1]._pvoid, 6, _dlist1, _p, _ode_matsol1, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
  cao = _ion_cao;
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_cleanup(Datum* _thread) {
   _nrn_destroy_sparseobj_thread(_thread[_spth1]._pvoid);
   _nrn_destroy_sparseobj_thread(_thread[_cvspth1]._pvoid);
 }
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 0, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 1, 2);
   nrn_update_ion_pointer(_k_sym, _ppvar, 2, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 3, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  n_o2_occupancy = n_o2_occupancy0;
  n_o1_occupancy = n_o1_occupancy0;
  n_c4_occupancy = n_c4_occupancy0;
  n_c3_occupancy = n_c3_occupancy0;
  n_c2_occupancy = n_c2_occupancy0;
  n_c1_occupancy = n_c1_occupancy0;
 {
   ek = - 84.69 ;
   temperature = celsius + 273.15 ;
   rates ( _threadargs_ ) ;
   rates ( _threadargs_ ) ;
    _ss_sparse_thread(&_thread[_spth1]._pvoid, 6, _slist1, _dlist1, _p, &t, dt, activation, _linmat1, _ppvar, _thread, _nt);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 6; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  cai = _ion_cai;
  cao = _ion_cao;
 initmodel(_p, _ppvar, _thread, _nt);
 }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   fopen = n_fcond ;
   g = fopen * conductance ;
   gion = gmax * fopen ;
   ik = gion * ( v - ek ) ;
   }
 _current += ik;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  cai = _ion_cai;
  cao = _ion_cao;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dik;
  _dik = ik;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ik += ik ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
  cai = _ion_cai;
  cao = _ion_cao;
 {  sparse_thread(&_thread[_spth1]._pvoid, 6, _slist1, _dlist1, _p, &t, dt, activation, _linmat1, _ppvar, _thread, _nt);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 6; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 } }}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(n_c2_occupancy) - _p;  _dlist1[0] = &(Dn_c2_occupancy) - _p;
 _slist1[1] = &(n_o2_occupancy) - _p;  _dlist1[1] = &(Dn_o2_occupancy) - _p;
 _slist1[2] = &(n_o1_occupancy) - _p;  _dlist1[2] = &(Dn_o1_occupancy) - _p;
 _slist1[3] = &(n_c4_occupancy) - _p;  _dlist1[3] = &(Dn_c4_occupancy) - _p;
 _slist1[4] = &(n_c3_occupancy) - _p;  _dlist1[4] = &(Dn_c3_occupancy) - _p;
 _slist1[5] = &(n_c1_occupancy) - _p;  _dlist1[5] = &(Dn_c1_occupancy) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
