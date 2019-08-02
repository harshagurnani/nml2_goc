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
 
#define nrn_init _nrn_init__Golgi_CALC2
#define _nrn_initial _nrn_initial__Golgi_CALC2
#define nrn_cur _nrn_cur__Golgi_CALC2
#define _nrn_current _nrn_current__Golgi_CALC2
#define nrn_jacob _nrn_jacob__Golgi_CALC2
#define nrn_state _nrn_state__Golgi_CALC2
#define _net_receive _net_receive__Golgi_CALC2 
#define rates rates__Golgi_CALC2 
#define states states__Golgi_CALC2 
 
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
#define restingConc _p[0]
#define decayConstant _p[1]
#define shellThickness _p[2]
#define Faraday _p[3]
#define AREA_SCALE _p[4]
#define LENGTH_SCALE _p[5]
#define effectiveRadius _p[6]
#define innerRadius _p[7]
#define shellVolume _p[8]
#define concentration _p[9]
#define extConcentration _p[10]
#define ca2i _p[11]
#define ca2o _p[12]
#define ica2 _p[13]
#define rate_concentration _p[14]
#define Dconcentration _p[15]
#define DextConcentration _p[16]
#define v _p[17]
#define _g _p[18]
#define _ion_ca2i	*_ppvar[0]._pval
#define _ion_ca2o	*_ppvar[1]._pval
#define _ion_ica2	*_ppvar[2]._pval
#define _style_ca2	*((int*)_ppvar[3]._pvoid)
#define diam	*_ppvar[4]._pval
#define area	*_ppvar[5]._pval
 
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
 "setdata_Golgi_CALC2", _hoc_setdata,
 "rates_Golgi_CALC2", _hoc_rates,
 0, 0
};
 /* declare global and static user variables */
 static int _thread1data_inuse = 0;
static double _thread1data[2];
#define _gth 0
#define iCa2_Golgi_CALC2 _thread1data[0]
#define iCa2 _thread[_gth]._pval[0]
#define initialExtConcentration initialExtConcentration_Golgi_CALC2
 double initialExtConcentration = 0;
#define initialConcentration initialConcentration_Golgi_CALC2
 double initialConcentration = 0;
#define surfaceArea_Golgi_CALC2 _thread1data[1]
#define surfaceArea _thread[_gth]._pval[1]
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "surfaceArea_Golgi_CALC2", "um2",
 "iCa2_Golgi_CALC2", "nA",
 "initialConcentration_Golgi_CALC2", "mM",
 "initialExtConcentration_Golgi_CALC2", "mM",
 "restingConc_Golgi_CALC2", "mM",
 "decayConstant_Golgi_CALC2", "ms",
 "shellThickness_Golgi_CALC2", "um",
 "Faraday_Golgi_CALC2", "C",
 "AREA_SCALE_Golgi_CALC2", "um2",
 "LENGTH_SCALE_Golgi_CALC2", "um",
 "concentration_Golgi_CALC2", "mM",
 "extConcentration_Golgi_CALC2", "mM",
 "effectiveRadius_Golgi_CALC2", "um",
 "innerRadius_Golgi_CALC2", "um",
 "shellVolume_Golgi_CALC2", "um3",
 0,0
};
 static double concentration0 = 0;
 static double delta_t = 0.01;
 static double extConcentration0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "surfaceArea_Golgi_CALC2", &surfaceArea_Golgi_CALC2,
 "iCa2_Golgi_CALC2", &iCa2_Golgi_CALC2,
 "initialConcentration_Golgi_CALC2", &initialConcentration_Golgi_CALC2,
 "initialExtConcentration_Golgi_CALC2", &initialExtConcentration_Golgi_CALC2,
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
 
#define _cvode_ieq _ppvar[6]._i
 static void _ode_synonym(int, double**, Datum**);
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.5.0",
"Golgi_CALC2",
 "restingConc_Golgi_CALC2",
 "decayConstant_Golgi_CALC2",
 "shellThickness_Golgi_CALC2",
 "Faraday_Golgi_CALC2",
 "AREA_SCALE_Golgi_CALC2",
 "LENGTH_SCALE_Golgi_CALC2",
 0,
 "effectiveRadius_Golgi_CALC2",
 "innerRadius_Golgi_CALC2",
 "shellVolume_Golgi_CALC2",
 0,
 "concentration_Golgi_CALC2",
 "extConcentration_Golgi_CALC2",
 0,
 0};
 static Symbol* _morphology_sym;
 extern Node* nrn_alloc_node_;
 static Symbol* _ca2_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 19, _prop);
 	/*initialize range parameters*/
 	restingConc = 5e-005;
 	decayConstant = 0.769231;
 	shellThickness = 0.20378;
 	Faraday = 0.0964853;
 	AREA_SCALE = 1e+012;
 	LENGTH_SCALE = 1e+006;
 	_prop->param = _p;
 	_prop->param_size = 19;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 7, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_morphology_sym);
 	_ppvar[4]._pval = &prop_ion->param[0]; /* diam */
 	_ppvar[5]._pval = &nrn_alloc_node_->_area; /* diam */
 prop_ion = need_memb(_ca2_sym);
 nrn_check_conc_write(_prop, prop_ion, 1);
 nrn_promote(prop_ion, 3, 0);
 	_ppvar[0]._pval = &prop_ion->param[1]; /* ca2i */
 	_ppvar[1]._pval = &prop_ion->param[2]; /* ca2o */
 	_ppvar[2]._pval = &prop_ion->param[3]; /* ica2 */
 	_ppvar[3]._pvoid = (void*)(&(prop_ion->dparam[0]._i)); /* iontype for ca2 */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _thread_mem_init(Datum*);
 static void _thread_cleanup(Datum*);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _Golgi_CALC2_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ca2", 2.0);
 	_morphology_sym = hoc_lookup("morphology");
 	_ca2_sym = hoc_lookup("ca2_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 2);
  _extcall_thread = (Datum*)ecalloc(1, sizeof(Datum));
  _thread_mem_init(_extcall_thread);
  _thread1data_inuse = 0;
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 1, _thread_mem_init);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 19, 7);
  hoc_register_dparam_semantics(_mechtype, 0, "ca2_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca2_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "ca2_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "#ca2_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "cvodeieq");
  hoc_register_dparam_semantics(_mechtype, 4, "diam");
  hoc_register_dparam_semantics(_mechtype, 5, "area");
 	nrn_writes_conc(_mechtype, 0);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_synonym(_mechtype, _ode_synonym);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 Golgi_CALC2 D:/Work/Comp Models/Learn Neuroml2/nml2_goc/Mechanisms/Golgi_CALC2.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "Mod file for component: Component(id=Golgi_CALC2 type=decayingPoolConcentrationModel)";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_threadargsproto_);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[1], _dlist1[1];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   rates ( _threadargs_ ) ;
   Dconcentration = rate_concentration ;
   ca2i = concentration ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 rates ( _threadargs_ ) ;
 Dconcentration = Dconcentration  / (1. - dt*( 0.0 )) ;
 ca2i = concentration ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   rates ( _threadargs_ ) ;
    concentration = concentration - dt*(- ( rate_concentration ) ) ;
   ca2i = concentration ;
   }
  return 0;
}
 
static int  rates ( _threadargsproto_ ) {
   surfaceArea = area ;
   iCa2 = - 1.0 * ( 0.01 ) * ica2 * surfaceArea ;
   effectiveRadius = LENGTH_SCALE * sqrt ( surfaceArea / ( AREA_SCALE * ( 4.0 * 3.14159 ) ) ) ;
   innerRadius = effectiveRadius - shellThickness ;
   shellVolume = ( 4.0 * ( effectiveRadius * effectiveRadius * effectiveRadius ) * 3.14159 / 3.0 ) - ( 4.0 * ( innerRadius * innerRadius * innerRadius ) * 3.14159 / 3.0 ) ;
   rate_concentration = iCa2 / ( 2.0 * Faraday * shellVolume ) - ( ( concentration - restingConc ) / decayConstant ) ;
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
 
static int _ode_count(int _type){ return 1;}
 
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
  ica2 = _ion_ica2;
  ca2i = _ion_ca2i;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  _ion_ca2i = ca2i;
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 1; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 static void _ode_synonym(int _cnt, double** _pp, Datum** _ppd) { 
	double* _p; Datum* _ppvar;
 	int _i; 
	for (_i=0; _i < _cnt; ++_i) {_p = _pp[_i]; _ppvar = _ppd[_i];
 _ion_ca2i =  concentration ;
 }}
 
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
  ica2 = _ion_ica2;
  ca2i = _ion_ca2i;
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_mem_init(Datum* _thread) {
  if (_thread1data_inuse) {_thread[_gth]._pval = (double*)ecalloc(2, sizeof(double));
 }else{
 _thread[_gth]._pval = _thread1data; _thread1data_inuse = 1;
 }
 }
 
static void _thread_cleanup(Datum* _thread) {
  if (_thread[_gth]._pval == _thread1data) {
   _thread1data_inuse = 0;
  }else{
   free((void*)_thread[_gth]._pval);
  }
 }
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca2_sym, _ppvar, 0, 1);
   nrn_update_ion_pointer(_ca2_sym, _ppvar, 1, 2);
   nrn_update_ion_pointer(_ca2_sym, _ppvar, 2, 3);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  concentration = concentration0;
  extConcentration = extConcentration0;
 {
   ca2i = initialConcentration ;
   ca2o = initialExtConcentration ;
   rates ( _threadargs_ ) ;
   rates ( _threadargs_ ) ;
   concentration = initialConcentration ;
   extConcentration = initialExtConcentration ;
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
  ca2i = _ion_ca2i;
  ca2o = _ion_ca2o;
  ica2 = _ion_ica2;
  ca2i = _ion_ca2i;
 initmodel(_p, _ppvar, _thread, _nt);
  _ion_ca2i = ca2i;
  nrn_wrote_conc(_ca2_sym, (&(_ion_ca2i)) - 1, _style_ca2);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{
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
  ica2 = _ion_ica2;
  ca2i = _ion_ca2i;
 {   states(_p, _ppvar, _thread, _nt);
  } {
   if ( concentration < 0.0 ) {
     concentration = 0.0 ;
     }
   }
  _ion_ca2i = ca2i;
}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(concentration) - _p;  _dlist1[0] = &(Dconcentration) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
