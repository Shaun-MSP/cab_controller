/*
************************************************************
* NOTE: Automatically generated file. DO NOT MODIFY!
************************************************************
*
* File: lp_filter.c
*
* Code generated from model             : 'cab1k_3_lvl_shaun'.
* Subsystem selected for code generation: 'lp_filter'.
*
* Schematic Editor version              : 2023.1 SP1
* C source code generated on            : 11-May-2023 @ 11:56:05 AM
*
*/

#include "UTILS/lp_filter.h"

//@cmp.def.start
// custom defines
//@cmp.def.end

// Model entry point functions
void lp_filter_step(lp_filter_ModelData *p_m_Data) {
    lp_filter_ExtIn *ext_In = (lp_filter_ExtIn *) p_m_Data->p_extIn;
    lp_filter_ExtOut *ext_Out = (lp_filter_ExtOut *) p_m_Data->p_extOut;
    lp_filter_ModelSinks *m_Sinks = (lp_filter_ModelSinks *) p_m_Data->p_Sinks;
    lp_filter_ModelStates *m_States = (lp_filter_ModelStates *) p_m_Data->p_States;
    //////////////////////////////////////////////////////////////////////////
    // Local variables
    //////////////////////////////////////////////////////////////////////////
    //@cmp.var.start
    real_t _lp_filter_low_pass_filter1__out;
    real_t _lp_filter_low_pass_filter1__previous_filtered_value;//@cmp.var.end
    //////////////////////////////////////////////////////////////////////////
    // Output block
    //////////////////////////////////////////////////////////////////////////
    //@cmp.out.block.start
    // Generated from the component: LP_Filter.Low-Pass Filter1
    _lp_filter_low_pass_filter1__previous_filtered_value = m_States->_lp_filter_low_pass_filter1__filtered_value;
    m_States->_lp_filter_low_pass_filter1__filtered_value = m_States->_lp_filter_low_pass_filter1__previous_in * (6.283185307179586 * 6.0 * 0.02) + _lp_filter_low_pass_filter1__previous_filtered_value * (1 - 6.283185307179586 * 6.0 * 0.02 );
    _lp_filter_low_pass_filter1__out = m_States->_lp_filter_low_pass_filter1__filtered_value;
    m_States->_lp_filter_low_pass_filter1__previous_in = ext_In->In2;
///////////////
    // Update sinks
    ///////////////
    ////////////////
    // Update output
    ////////////////
    ext_Out->Out1 = _lp_filter_low_pass_filter1__out;
    //@cmp.out.block.end
    //////////////////////////////////////////////////////////////////////////
    // Update block
    //////////////////////////////////////////////////////////////////////////
    //@cmp.update.block.start
//@cmp.update.block.end
}

void lp_filter_init(lp_filter_ModelData *p_m_Data) {
    //@cmp.init.block.start
    lp_filter_ModelStates *m_States = (lp_filter_ModelStates *) p_m_Data->p_States;
    m_States->_lp_filter_low_pass_filter1__filtered_value = 0.0 / (1 - 6.283185307179586 * 6.0 * 0.02 );
    m_States->_lp_filter_low_pass_filter1__previous_in = 0x0;
    //@cmp.init.block.end
}
