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
 
#define nrn_init _nrn_init__Golgi_hcn1
#define _nrn_initial _nrn_initial__Golgi_hcn1
#define nrn_cur _nrn_cur__Golgi_hcn1
#define _nrn_current _nrn_current__Golgi_hcn1
#define nrn_jacob _nrn_jacob__Golgi_hcn1
#define nrn_state _nrn_state__Golgi_hcn1
#define _net_receive _net_receive__Golgi_hcn1 
#define _f_rate _f_rate__Golgi_hcn1 
#define rate rate__Golgi_hcn1 
#define state state__Golgi_hcn1 
 
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
#define gbar _p[0]
#define Erev _p[1]
#define ih _p[2]
#define g _p[3]
#define o_fast_inf _p[4]
#define o_slow_inf _p[5]
#define tau_f _p[6]
#define tau_s _p[7]
#define o_fast _p[8]
#define o_slow _p[9]
#define Do_fast _p[10]
#define Do_slow _p[11]
#define v _p[12]
#define _g _p[13]
 
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
 static void _hoc_o_inf(void);
 static void _hoc_q10(void);
 static void _hoc_rate(void);
 static void _hoc_r(void);
 static void _hoc_tau(void);
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
 "setdata_Golgi_hcn1", _hoc_setdata,
 "o_inf_Golgi_hcn1", _hoc_o_inf,
 "q10_Golgi_hcn1", _hoc_q10,
 "rate_Golgi_hcn1", _hoc_rate,
 "r_Golgi_hcn1", _hoc_r,
 "tau_Golgi_hcn1", _hoc_tau,
 0, 0
};
#define o_inf o_inf_Golgi_hcn1
#define q10 q10_Golgi_hcn1
#define r r_Golgi_hcn1
#define tau tau_Golgi_hcn1
 extern double o_inf( _threadargsprotocomma_ double , double , double );
 extern double q10( _threadargsprotocomma_ double );
 extern double r( _threadargsprotocomma_ double );
 extern double tau( _threadargsprotocomma_ double , double , double , double );
 
static void _check_rate(double*, Datum*, Datum*, _NrnThread*); 
static void _check_table_thread(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, int _type) {
   _check_rate(_p, _ppvar, _thread, _nt);
 }
 #define _zq _thread[0]._pval[0]
 /* declare global and static user variables */
#define Ehalf Ehalf_Golgi_hcn1
 double Ehalf = -72.49;
#define c c_Golgi_hcn1
 double c = 0.11305;
#define q_10 q_10_Golgi_hcn1
 double q_10 = 3;
#define rB rB_Golgi_hcn1
 double rB = 0.97596;
#define rA rA_Golgi_hcn1
 double rA = 0.002096;
#define tEs tEs_Golgi_hcn1
 double tEs = 2.30259;
#define tDs tDs_Golgi_hcn1
 double tDs = -4.056;
#define tCs tCs_Golgi_hcn1
 double tCs = 0.01451;
#define tEf tEf_Golgi_hcn1
 double tEf = 2.30259;
#define tDf tDf_Golgi_hcn1
 double tDf = -3.368;
#define tCf tCf_Golgi_hcn1
 double tCf = 0.01371;
#define usetable usetable_Golgi_hcn1
 double usetable = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "usetable_Golgi_hcn1", 0, 1,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "Ehalf_Golgi_hcn1", "mV",
 "c_Golgi_hcn1", "/mV",
 "rA_Golgi_hcn1", "/mV",
 "rB_Golgi_hcn1", "1",
 "tCf_Golgi_hcn1", "1",
 "tDf_Golgi_hcn1", "mV",
 "tEf_Golgi_hcn1", "/mV",
 "tCs_Golgi_hcn1", "1",
 "tDs_Golgi_hcn1", "mV",
 "tEs_Golgi_hcn1", "/mV",
 "gbar_Golgi_hcn1", "S/cm2",
 "Erev_Golgi_hcn1", "mV",
 "ih_Golgi_hcn1", "mA/cm2",
 "g_Golgi_hcn1", "S/cm2",
 "tau_f_Golgi_hcn1", "ms",
 "tau_s_Golgi_hcn1", "ms",
 0,0
};
 static double delta_t = 0.01;
 static double o_slow0 = 0;
 static double o_fast0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "q_10_Golgi_hcn1", &q_10_Golgi_hcn1,
 "Ehalf_Golgi_hcn1", &Ehalf_Golgi_hcn1,
 "c_Golgi_hcn1", &c_Golgi_hcn1,
 "rA_Golgi_hcn1", &rA_Golgi_hcn1,
 "rB_Golgi_hcn1", &rB_Golgi_hcn1,
 "tCf_Golgi_hcn1", &tCf_Golgi_hcn1,
 "tDf_Golgi_hcn1", &tDf_Golgi_hcn1,
 "tEf_Golgi_hcn1", &tEf_Golgi_hcn1,
 "tCs_Golgi_hcn1", &tCs_Golgi_hcn1,
 "tDs_Golgi_hcn1", &tDs_Golgi_hcn1,
 "tEs_Golgi_hcn1", &tEs_Golgi_hcn1,
 "usetable_Golgi_hcn1", &usetable_Golgi_hcn1,
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
 
#define _cvode_ieq _ppvar[0]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.5.0",
"Golgi_hcn1",
 "gbar_Golgi_hcn1",
 "Erev_Golgi_hcn1",
 0,
 "ih_Golgi_hcn1",
 "g_Golgi_hcn1",
 "o_fast_inf_Golgi_hcn1",
 "o_slow_inf_Golgi_hcn1",
 "tau_f_Golgi_hcn1",
 "tau_s_Golgi_hcn1",
 0,
 "o_fast_Golgi_hcn1",
 "o_slow_Golgi_hcn1",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 14, _prop);
 	/*initialize range parameters*/
 	gbar = 5e-005;
 	Erev = -20;
 	_prop->param = _p;
 	_prop->param_size = 14;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 1, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _thread_mem_init(Datum*);
 static void _thread_cleanup(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _Golgi_hcn1_reg() {
	int _vectorized = 1;
  _initlists();
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 2);
  _extcall_thread = (Datum*)ecalloc(1, sizeof(Datum));
  _thread_mem_init(_extcall_thread);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 1, _thread_mem_init);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
     _nrn_thread_table_reg(_mechtype, _check_table_thread);
  hoc_register_prop_size(_mechtype, 14, 1);
  hoc_register_dparam_semantics(_mechtype, 0, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 Golgi_hcn1 D:/Work/Comp Models/Learn Neuroml2/nml2_goc/Golgi_hcn1.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 /*Top LOCAL _zq */
 static double *_t_o_fast_inf;
 static double *_t_o_slow_inf;
 static double *_t_tau_f;
 static double *_t_tau_s;
static int _reset;
static char *modelname = "Cerebellum Golgi Cell HCN1 Model";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int _f_rate(_threadargsprotocomma_ double);
static int rate(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static void _n_rate(_threadargsprotocomma_ double _lv);
 static int _slist1[2], _dlist1[2];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   rate ( _threadargscomma_ v ) ;
   Do_fast = ( o_fast_inf - o_fast ) / tau_f ;
   Do_slow = ( o_slow_inf - o_slow ) / tau_s ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 rate ( _threadargscomma_ v ) ;
 Do_fast = Do_fast  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_f )) ;
 Do_slow = Do_slow  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_s )) ;
  return 0;
}
 /*END CVODE*/
 static int state (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   rate ( _threadargscomma_ v ) ;
    o_fast = o_fast + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_f)))*(- ( ( ( o_fast_inf ) ) / tau_f ) / ( ( ( ( - 1.0 ) ) ) / tau_f ) - o_fast) ;
    o_slow = o_slow + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_s)))*(- ( ( ( o_slow_inf ) ) / tau_s ) / ( ( ( ( - 1.0 ) ) ) / tau_s ) - o_slow) ;
   }
  return 0;
}
 
double r ( _threadargsprotocomma_ double _lpotential ) {
   double _lr;
  _lr = rA * _lpotential + rB ;
    
return _lr;
 }
 
static void _hoc_r(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  r ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double tau ( _threadargsprotocomma_ double _lpotential , double _lt1 , double _lt2 , double _lt3 ) {
   double _ltau;
  _ltau = exp ( ( ( _lt1 * _lpotential ) - _lt2 ) * _lt3 ) ;
    
return _ltau;
 }
 
static void _hoc_tau(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  tau ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) );
 hoc_retpushx(_r);
}
 
double o_inf ( _threadargsprotocomma_ double _lpotential , double _lEhalf , double _lc ) {
   double _lo_inf;
  _lo_inf = 1.0 / ( 1.0 + exp ( ( _lpotential - _lEhalf ) * _lc ) ) ;
    
return _lo_inf;
 }
 
static void _hoc_o_inf(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  o_inf ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) , *getarg(3) );
 hoc_retpushx(_r);
}
 
double q10 ( _threadargsprotocomma_ double _lcelsius ) {
   double _lq10;
  _lq10 = exp ( 1.0986 * ( ( _lcelsius - 33.0 ) / 10.0 ) ) ;
    
return _lq10;
 }
 
static void _hoc_q10(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  q10 ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 static double _mfac_rate, _tmin_rate;
  static void _check_rate(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  static int _maktable=1; int _i, _j, _ix = 0;
  double _xi, _tmax;
  static double _sav_celsius;
  if (!usetable) {return;}
  if (_sav_celsius != celsius) { _maktable = 1;}
  if (_maktable) { double _x, _dx; _maktable=0;
   _tmin_rate =  - 100.0 ;
   _tmax =  100.0 ;
   _dx = (_tmax - _tmin_rate)/20000.; _mfac_rate = 1./_dx;
   for (_i=0, _x=_tmin_rate; _i < 20001; _x += _dx, _i++) {
    _f_rate(_p, _ppvar, _thread, _nt, _x);
    _t_o_fast_inf[_i] = o_fast_inf;
    _t_o_slow_inf[_i] = o_slow_inf;
    _t_tau_f[_i] = tau_f;
    _t_tau_s[_i] = tau_s;
   }
   _sav_celsius = celsius;
  }
 }

 static int rate(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _lv) { 
#if 0
_check_rate(_p, _ppvar, _thread, _nt);
#endif
 _n_rate(_p, _ppvar, _thread, _nt, _lv);
 return 0;
 }

 static void _n_rate(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _lv){ int _i, _j;
 double _xi, _theta;
 if (!usetable) {
 _f_rate(_p, _ppvar, _thread, _nt, _lv); return; 
}
 _xi = _mfac_rate * (_lv - _tmin_rate);
 if (isnan(_xi)) {
  o_fast_inf = _xi;
  o_slow_inf = _xi;
  tau_f = _xi;
  tau_s = _xi;
  return;
 }
 if (_xi <= 0.) {
 o_fast_inf = _t_o_fast_inf[0];
 o_slow_inf = _t_o_slow_inf[0];
 tau_f = _t_tau_f[0];
 tau_s = _t_tau_s[0];
 return; }
 if (_xi >= 20000.) {
 o_fast_inf = _t_o_fast_inf[20000];
 o_slow_inf = _t_o_slow_inf[20000];
 tau_f = _t_tau_f[20000];
 tau_s = _t_tau_s[20000];
 return; }
 _i = (int) _xi;
 _theta = _xi - (double)_i;
 o_fast_inf = _t_o_fast_inf[_i] + _theta*(_t_o_fast_inf[_i+1] - _t_o_fast_inf[_i]);
 o_slow_inf = _t_o_slow_inf[_i] + _theta*(_t_o_slow_inf[_i+1] - _t_o_slow_inf[_i]);
 tau_f = _t_tau_f[_i] + _theta*(_t_tau_f[_i+1] - _t_tau_f[_i]);
 tau_s = _t_tau_s[_i] + _theta*(_t_tau_s[_i+1] - _t_tau_s[_i]);
 }

 
static int  _f_rate ( _threadargsprotocomma_ double _lv ) {
   o_fast_inf = r ( _threadargscomma_ _lv ) * o_inf ( _threadargscomma_ _lv , Ehalf , c ) ;
   o_slow_inf = ( 1.0 - r ( _threadargscomma_ _lv ) ) * o_inf ( _threadargscomma_ _lv , Ehalf , c ) ;
   tau_f = tau ( _threadargscomma_ _lv , tCf , tDf , tEf ) ;
   tau_s = tau ( _threadargscomma_ _lv , tCs , tDs , tEs ) ;
    return 0; }
 
static void _hoc_rate(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 
#if 1
 _check_rate(_p, _ppvar, _thread, _nt);
#endif
 _r = 1.;
 rate ( _p, _ppvar, _thread, _nt, *getarg(1) );
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
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_mem_init(Datum* _thread) {
   _thread[0]._pval = (double*)ecalloc(1, sizeof(double));
 }
 
static void _thread_cleanup(Datum* _thread) {
   free((void*)(_thread[0]._pval));
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  o_slow = o_slow0;
  o_fast = o_fast0;
 {
   _zq = pow( q_10 , ( ( celsius - 33.0 ) / 10.0 ) ) ;
   rate ( _threadargscomma_ v ) ;
   o_fast = o_fast_inf ;
   o_slow = o_slow_inf ;
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
 _check_rate(_p, _ppvar, _thread, _nt);
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
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   g = gbar * ( o_fast + o_slow ) ;
   ih = g * ( v - Erev ) ;
   }
 _current += ih;

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
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
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
 {   state(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(o_fast) - _p;  _dlist1[0] = &(Do_fast) - _p;
 _slist1[1] = &(o_slow) - _p;  _dlist1[1] = &(Do_slow) - _p;
   _t_o_fast_inf = makevector(20001*sizeof(double));
   _t_o_slow_inf = makevector(20001*sizeof(double));
   _t_tau_f = makevector(20001*sizeof(double));
   _t_tau_s = makevector(20001*sizeof(double));
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
