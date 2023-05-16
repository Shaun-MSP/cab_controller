/*
************************************************************
* NOTE: Automatically generated file. DO NOT MODIFY!
************************************************************
*
* File: lp_filter.h
*
* Code generated from model             : 'cab1k_3_lvl_shaun'.
* Subsystem selected for code generation: 'lp_filter'.
*
* Schematic Editor version              : 2023.1 SP1
* C source code generated on            : 11-May-2023 @ 11:56:05 AM
*
*/

#include "types.h"

// External input
typedef struct {
    // Generated from the component: LP_Filter.In2
    real_t In2;
} lp_filter_ExtIn;


// External output
typedef struct {
    // Generated from the component: LP_Filter.Out1
    real_t Out1;
} lp_filter_ExtOut;

// Sinks
typedef struct {
} lp_filter_ModelSinks;

// States
typedef struct {
    // Generated from the component: LP_Filter.Low-Pass Filter1
    real_t _lp_filter_low_pass_filter1__filtered_value;
    real_t _lp_filter_low_pass_filter1__previous_in;
} lp_filter_ModelStates;

// Model data structure
typedef struct {
    lp_filter_ExtIn *p_extIn;
    lp_filter_ExtOut *p_extOut;
    lp_filter_ModelSinks *p_Sinks;
    lp_filter_ModelStates *p_States;
} lp_filter_ModelData;

// External input
extern lp_filter_ExtIn lp_filter_ext_In;

// External output
extern lp_filter_ExtOut lp_filter_ext_Out;

// Sinks
extern lp_filter_ModelSinks lp_filter_m_Sinks;

// States
extern lp_filter_ModelStates lp_filter_m_States;

// Model data structure
extern lp_filter_ModelData lp_filter_m_Data;

// Model entry point functions
extern void lp_filter_init(lp_filter_ModelData *p_m_Data);
extern void lp_filter_step(lp_filter_ModelData *p_m_Data);
