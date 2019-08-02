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
 
#define nrn_init _nrn_init__GolgiCaLVA
#define _nrn_initial _nrn_initial__GolgiCaLVA
#define nrn_cur _nrn_cur__GolgiCaLVA
#define _nrn_current _nrn_current__GolgiCaLVA
#define nrn_jacob _nrn_jacob__GolgiCaLVA
#define nrn_state _nrn_state__GolgiCaLVA
#define _net_receive _net_receive__GolgiCaLVA 
#define _f_evaluate_fct _f_evaluate_fct__GolgiCaLVA 
#define ca2state ca2state__GolgiCaLVA 
#define evaluate_fct evaluate_fct__GolgiCaLVA 
 
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
#define gca2bar _p[0]
#define shift _p[1]
#define v0_m_inf _p[2]
#define v0_h_inf _p[3]
#define k_m_inf _p[4]
#define k_h_inf _p[5]
#define C_tau_m _p[6]
#define A_tau_m _p[7]
#define v0_tau_m1 _p[8]
#define v0_tau_m2 _p[9]
#define k_tau_m1 _p[10]
#define k_tau_m2 _p[11]
#define C_tau_h _p[12]
#define A_tau_h _p[13]
#define v0_tau_h1 _p[14]
#define v0_tau_h2 _p[15]
#define k_tau_h1 _p[16]
#define k_tau_h2 _p[17]
#define ica2 _p[18]
#define ca2rev _p[19]
#define g _p[20]
#define m_inf _p[21]
#define tau_m _p[22]
#define h_inf _p[23]
#define tau_h _p[24]
#define phi_m _p[25]
#define phi_h _p[26]
#define m _p[27]
#define h _p[28]
#define ca2i _p[29]
#define ca2o _p[30]
#define Dm _p[31]
#define Dh _p[32]
#define v _p[33]
#define _g _p[34]
#define _ion_ca2i	*_ppvar[0]._pval
#define _ion_ca2o	*_ppvar[1]._pval
#define _ion_ica2	*_ppvar[2]._pval
#define _ion_dica2dv	*_ppvar[3]._pval
 
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
 static void _hoc_evaluate_fct(void);
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
 "setdata_GolgiCaLVA", _hoc_setdata,
 "evaluate_fct_GolgiCaLVA", _hoc_evaluate_fct,
 0, 0
};
 
static void _check_evaluate_fct(double*, Datum*, Datum*, _NrnThread*); 
static void _check_table_thread(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, int _type) {
   _check_evaluate_fct(_p, _ppvar, _thread, _nt);
 }
 /* declare global and static user variables */
#define eca2 eca2_GolgiCaLVA
 double eca2 = 0;
#define usetable usetable_GolgiCaLVA
 double usetable = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "usetable_GolgiCaLVA", 0, 1,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "eca2_GolgiCaLVA", "mV",
 "gca2bar_GolgiCaLVA", "mho/cm2",
 "shift_GolgiCaLVA", "mV",
 "v0_m_inf_GolgiCaLVA", "mV",
 "v0_h_inf_GolgiCaLVA", "mV",
 "k_m_inf_GolgiCaLVA", "mV",
 "k_h_inf_GolgiCaLVA", "mv",
 "v0_tau_m1_GolgiCaLVA", "mV",
 "v0_tau_m2_GolgiCaLVA", "mV",
 "k_tau_m1_GolgiCaLVA", "mV",
 "k_tau_m2_GolgiCaLVA", "mV",
 "v0_tau_h1_GolgiCaLVA", "mV",
 "v0_tau_h2_GolgiCaLVA", "mV",
 "k_tau_h1_GolgiCaLVA", "mV",
 "k_tau_h2_GolgiCaLVA", "mV",
 "ica2_GolgiCaLVA", "mA/cm2",
 "ca2rev_GolgiCaLVA", "mV",
 "g_GolgiCaLVA", "mho/cm2",
 "tau_m_GolgiCaLVA", "ms",
 "tau_h_GolgiCaLVA", "ms",
 0,0
};
 static double delta_t = 1;
 static double h0 = 0;
 static double m0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "eca2_GolgiCaLVA", &eca2_GolgiCaLVA,
 "usetable_GolgiCaLVA", &usetable_GolgiCaLVA,
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
"GolgiCaLVA",
 "gca2bar_GolgiCaLVA",
 "shift_GolgiCaLVA",
 "v0_m_inf_GolgiCaLVA",
 "v0_h_inf_GolgiCaLVA",
 "k_m_inf_GolgiCaLVA",
 "k_h_inf_GolgiCaLVA",
 "C_tau_m_GolgiCaLVA",
 "A_tau_m_GolgiCaLVA",
 "v0_tau_m1_GolgiCaLVA",
 "v0_tau_m2_GolgiCaLVA",
 "k_tau_m1_GolgiCaLVA",
 "k_tau_m2_GolgiCaLVA",
 "C_tau_h_GolgiCaLVA",
 "A_tau_h_GolgiCaLVA",
 "v0_tau_h1_GolgiCaLVA",
 "v0_tau_h2_GolgiCaLVA",
 "k_tau_h1_GolgiCaLVA",
 "k_tau_h2_GolgiCaLVA",
 0,
 "ica2_GolgiCaLVA",
 "ca2rev_GolgiCaLVA",
 "g_GolgiCaLVA",
 "m_inf_GolgiCaLVA",
 "tau_m_GolgiCaLVA",
 "h_inf_GolgiCaLVA",
 "tau_h_GolgiCaLVA",
 "phi_m_GolgiCaLVA",
 "phi_h_GolgiCaLVA",
 0,
 "m_GolgiCaLVA",
 "h_GolgiCaLVA",
 0,
 0};
 static Symbol* _ca2_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 35, _prop);
 	/*initialize range parameters*/
 	gca2bar = 0.00025;
 	shift = 2;
 	v0_m_inf = -50;
 	v0_h_inf = -78;
 	k_m_inf = -7.4;
 	k_h_inf = 5;
 	C_tau_m = 3;
 	A_tau_m = 1;
 	v0_tau_m1 = -25;
 	v0_tau_m2 = -100;
 	k_tau_m1 = 10;
 	k_tau_m2 = -15;
 	C_tau_h = 85;
 	A_tau_h = 1;
 	v0_tau_h1 = -46;
 	v0_tau_h2 = -405;
 	k_tau_h1 = 4;
 	k_tau_h2 = -50;
 	_prop->param = _p;
 	_prop->param_size = 35;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca2_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[0]._pval = &prop_ion->param[1]; /* ca2i */
 	_ppvar[1]._pval = &prop_ion->param[2]; /* ca2o */
 	_ppvar[2]._pval = &prop_ion->param[3]; /* ica2 */
 	_ppvar[3]._pval = &prop_ion->param[4]; /* _ion_dica2dv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _Golgi_Ca_LVA_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ca2", 2.0);
 	_ca2_sym = hoc_lookup("ca2_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
     _nrn_thread_table_reg(_mechtype, _check_table_thread);
  hoc_register_prop_size(_mechtype, 35, 5);
  hoc_register_dparam_semantics(_mechtype, 0, "ca2_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca2_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "ca2_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca2_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 GolgiCaLVA D:/Work/Comp Models/Learn Neuroml2/nml2_goc/Golgi_Ca_LVA.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96485.3;
 static double R = 8.3145;
 static double *_t_m_inf;
 static double *_t_tau_m;
 static double *_t_h_inf;
 static double *_t_tau_h;
static int _reset;
static char *modelname = "Low threshold calcium current Cerebellum Golgi Cell Model";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int _f_evaluate_fct(_threadargsprotocomma_ double);
static int evaluate_fct(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static void _n_evaluate_fct(_threadargsprotocomma_ double _lv);
 static int _slist1[2], _dlist1[2];
 static int ca2state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   evaluate_fct ( _threadargscomma_ v ) ;
   Dm = ( m_inf - m ) / tau_m ;
   Dh = ( h_inf - h ) / tau_h ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 evaluate_fct ( _threadargscomma_ v ) ;
 Dm = Dm  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_m )) ;
 Dh = Dh  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_h )) ;
  return 0;
}
 /*END CVODE*/
 static int ca2state (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   evaluate_fct ( _threadargscomma_ v ) ;
    m = m + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_m)))*(- ( ( ( m_inf ) ) / tau_m ) / ( ( ( ( - 1.0 ) ) ) / tau_m ) - m) ;
    h = h + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_h)))*(- ( ( ( h_inf ) ) / tau_h ) / ( ( ( ( - 1.0 ) ) ) / tau_h ) - h) ;
   }
  return 0;
}
 static double _mfac_evaluate_fct, _tmin_evaluate_fct;
  static void _check_evaluate_fct(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  static int _maktable=1; int _i, _j, _ix = 0;
  double _xi, _tmax;
  static double _sav_shift;
  static double _sav_phi_m;
  static double _sav_phi_h;
  if (!usetable) {return;}
  if (_sav_shift != shift) { _maktable = 1;}
  if (_sav_phi_m != phi_m) { _maktable = 1;}
  if (_sav_phi_h != phi_h) { _maktable = 1;}
  if (_maktable) { double _x, _dx; _maktable=0;
   _tmin_evaluate_fct =  - 100.0 ;
   _tmax =  100.0 ;
   _dx = (_tmax - _tmin_evaluate_fct)/20000.; _mfac_evaluate_fct = 1./_dx;
   for (_i=0, _x=_tmin_evaluate_fct; _i < 20001; _x += _dx, _i++) {
    _f_evaluate_fct(_p, _ppvar, _thread, _nt, _x);
    _t_m_inf[_i] = m_inf;
    _t_tau_m[_i] = tau_m;
    _t_h_inf[_i] = h_inf;
    _t_tau_h[_i] = tau_h;
   }
   _sav_shift = shift;
   _sav_phi_m = phi_m;
   _sav_phi_h = phi_h;
  }
 }

 static int evaluate_fct(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _lv) { 
#if 0
_check_evaluate_fct(_p, _ppvar, _thread, _nt);
#endif
 _n_evaluate_fct(_p, _ppvar, _thread, _nt, _lv);
 return 0;
 }

 static void _n_evaluate_fct(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _lv){ int _i, _j;
 double _xi, _theta;
 if (!usetable) {
 _f_evaluate_fct(_p, _ppvar, _thread, _nt, _lv); return; 
}
 _xi = _mfac_evaluate_fct * (_lv - _tmin_evaluate_fct);
 if (isnan(_xi)) {
  m_inf = _xi;
  tau_m = _xi;
  h_inf = _xi;
  tau_h = _xi;
  return;
 }
 if (_xi <= 0.) {
 m_inf = _t_m_inf[0];
 tau_m = _t_tau_m[0];
 h_inf = _t_h_inf[0];
 tau_h = _t_tau_h[0];
 return; }
 if (_xi >= 20000.) {
 m_inf = _t_m_inf[20000];
 tau_m = _t_tau_m[20000];
 h_inf = _t_h_inf[20000];
 tau_h = _t_tau_h[20000];
 return; }
 _i = (int) _xi;
 _theta = _xi - (double)_i;
 m_inf = _t_m_inf[_i] + _theta*(_t_m_inf[_i+1] - _t_m_inf[_i]);
 tau_m = _t_tau_m[_i] + _theta*(_t_tau_m[_i+1] - _t_tau_m[_i]);
 h_inf = _t_h_inf[_i] + _theta*(_t_h_inf[_i+1] - _t_h_inf[_i]);
 tau_h = _t_tau_h[_i] + _theta*(_t_tau_h[_i+1] - _t_tau_h[_i]);
 }

 
static int  _f_evaluate_fct ( _threadargsprotocomma_ double _lv ) {
   phi_m = pow( 5.0 , ( ( celsius - 24.0 ) / 10.0 ) ) ;
   phi_h = pow( 3.0 , ( ( celsius - 24.0 ) / 10.0 ) ) ;
   m_inf = 1.0 / ( 1.0 + exp ( ( _lv + shift - v0_m_inf ) / k_m_inf ) ) ;
   h_inf = 1.0 / ( 1.0 + exp ( ( _lv + shift - v0_h_inf ) / k_h_inf ) ) ;
   tau_m = ( C_tau_m + A_tau_m / ( exp ( ( _lv + shift - v0_tau_m1 ) / k_tau_m1 ) + exp ( ( _lv + shift - v0_tau_m2 ) / k_tau_m2 ) ) ) / phi_m ;
   tau_h = ( C_tau_h + A_tau_h / ( exp ( ( _lv + shift - v0_tau_h1 ) / k_tau_h1 ) + exp ( ( _lv + shift - v0_tau_h2 ) / k_tau_h2 ) ) ) / phi_h ;
    return 0; }
 
static void _hoc_evaluate_fct(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 
#if 1
 _check_evaluate_fct(_p, _ppvar, _thread, _nt);
#endif
 _r = 1.;
 evaluate_fct ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ca2i = _ion_ca2i;
  ca2o = _ion_ca2o;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
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
  ca2i = _ion_ca2i;
  ca2o = _ion_ca2o;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca2_sym, _ppvar, 0, 1);
   nrn_update_ion_pointer(_ca2_sym, _ppvar, 1, 2);
   nrn_update_ion_pointer(_ca2_sym, _ppvar, 2, 3);
   nrn_update_ion_pointer(_ca2_sym, _ppvar, 3, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  h = h0;
  m = m0;
 {
   evaluate_fct ( _threadargscomma_ v ) ;
   m = m_inf ;
   h = h_inf ;
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

#if 0
 _check_evaluate_fct(_p, _ppvar, _thread, _nt);
#endif
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
  ca2i = _ion_ca2i;
  ca2o = _ion_ca2o;
 initmodel(_p, _ppvar, _thread, _nt);
 }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   ca2rev = ( 1e3 ) * ( R * ( celsius + 273.15 ) ) / ( 2.0 * FARADAY ) * log ( ca2o / ca2i ) ;
   g = gca2bar * m * m * h ;
   ica2 = gca2bar * m * m * h * ( v - ca2rev ) ;
   }
 _current += ica2;

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
  ca2i = _ion_ca2i;
  ca2o = _ion_ca2o;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dica2;
  _dica2 = ica2;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dica2dv += (_dica2 - ica2)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica2 += ica2 ;
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
  ca2i = _ion_ca2i;
  ca2o = _ion_ca2o;
 {   ca2state(_p, _ppvar, _thread, _nt);
  } }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(m) - _p;  _dlist1[0] = &(Dm) - _p;
 _slist1[1] = &(h) - _p;  _dlist1[1] = &(Dh) - _p;
   _t_m_inf = makevector(20001*sizeof(double));
   _t_tau_m = makevector(20001*sizeof(double));
   _t_h_inf = makevector(20001*sizeof(double));
   _t_tau_h = makevector(20001*sizeof(double));
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
