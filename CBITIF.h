/** \file
 *
 *  Specifies the interface details for the CBIT component in the
 *  BITModule.
 *
 *  CBIT.h should only be included by CBIT.cpp
 *  Other classes should include CBITIF.h
 *
 *  Copyright (c) 2007,2008,2009 MBARI
 *  MBARI Proprietary Information.  All Rights Reserved
 */

#ifndef CBITIF_H_
#define CBITIF_H_

#include "data/ElementURI.h"

/**
 *  Specifies the interface details for the CBIT component in the
 *  BITModule.
 *
 *  CBIT.h should only be included by CBIT.cpp
 *  Other classes should include CBITIF.h
 *
 *  \ingroup modules_builtintest
 */
namespace CBITIF
{
/// Static const for component name
/// (Can be used by other components)
static const Str NAME( "CBIT" );

// Include common ConfigURIs -- must be in namespace and follow definition of NAME
#include "component/HasLoadAtStartupIF.h"
#include "component/HasHardwareIF.h"

// TODO: change these static const str to DataURI, etc.
/// Static consts for input settings
static const DataURI CLEAR_FAULT_CMD( NAME, "clearFaultCmd", Units::ENUM );
static const DataURI CLEAR_LEAK_FAULT_CMD( NAME, "clearLeakFaultCmd", Units::ENUM );

static const DataURI SHOREPOWER_ON( NAME, "shorePowerOn", Units::BOOL );

static const DataURI GF_ACTIVE_STATE( NAME, "GFActive", Units::BOOL );
static const DataURI GFCHAN0_READING( NAME, "GFCHAN0Current", Units::MILLIAMPERE );
static const DataURI GFCHAN1_READING( NAME, "GFCHAN1Current", Units::MILLIAMPERE );
static const DataURI GFCHAN2_READING( NAME, "GFCHAN2Current", Units::MILLIAMPERE );
static const DataURI GFCHAN4_READING( NAME, "GFCHAN4Current", Units::MILLIAMPERE );
static const DataURI GFCHAN5_READING( NAME, "GFCHAN5Current", Units::MILLIAMPERE );
static const DataURI GFCHANOPEN_READING( NAME, "GFCHANOpenCurrent", Units::MILLIAMPERE );



static const DataURI EMPERICAL_FAULT_READING( NAME, "empericalClassifierFaultDetected", Units::BOOL );
static const DataURI BINNED_DEPTH_RATE_READING( NAME, "binnedDepthRate", Units::METER_PER_SECOND );

static const ConfigURI ABORT_DEPTH_CFG( NAME, "abortDepth", Units::METER );                        // Depth at which we drop the weight. Should be greater than all depth envelopes
static const ConfigURI STOP_DEPTH_CFG( NAME, "stopDepth", Units::METER );                          // Depth at which we stop the mission. Should be greater than all depth envelopes and less than abort depth

static const ConfigURI HUMIDITY_THRESHOLD_CFG( NAME, "humidityThreshold", Units::PERCENT );    // relative humidity
static const ConfigURI PRESSURE_THRESHOLD_CFG( NAME, "pressureThreshold", Units::POUND_PER_SQUARE_INCH ); // Onboard pressure must measure greater than this offset from 1 ATM
static const ConfigURI TEMP_THRESHOLD_CFG( NAME, "tempThreshold", Units::CELSIUS );            // Only need to use kelvin for temperature deltas

static const ConfigURI VEHICLE_OPEN_CFG( NAME, "vehicleOpen", Units::BOOL );
static const ConfigURI ABORT_DEPTH_TIMEOUT_CFG( NAME, "abortDepthTimeout", Units::SECOND );      // If this time has elapsed and we're still below abort depth, drop the weight, end the mission.
static const ConfigURI BATT_FAIL_REPORT_CFG( NAME, "battFailReport", Units::COUNT );             // Report main battery failover at this multiple
static const ConfigURI ENV_TIMEOUT_CFG( NAME, "envTimeout", Units::SECOND );                     // If this time has elapsed and pressure or humidity are still above threshold, end the mission.
static const ConfigURI RUN_FAULT_CLASSIFIER( NAME, "runFaultClassifier", Units::BOOL );          // Runs the empirical fault classifier as part of CBIT if true
static const ConfigURI RUN_ELEV_OFFSET_CALCULATOR( NAME, "runElevOffsetCalc", Units::BOOL );     // Runs the empirical fault classifier's elevator offset calculator as part of CBIT if true
    
static const ConfigURI BATT_TEMP_THRESHOLD_CFG( NAME, "battTempThreshold", Units::CELSIUS );     // The allowable threshold for battery temperature

static const ConfigURI GFCHAN0_THRESHOLD_CFG( NAME, "gfChan0_Threshold", Units::MILLIAMPERE ); // Current above baseline that triggers a ground fault
static const ConfigURI GFCHAN1_THRESHOLD_CFG( NAME, "gfChan1_Threshold", Units::MILLIAMPERE ); // Current above baseline that triggers a ground fault
static const ConfigURI GFCHAN2_THRESHOLD_CFG( NAME, "gfChan2_Threshold", Units::MILLIAMPERE ); // Current above baseline that triggers a ground fault
static const ConfigURI GFCHAN4_THRESHOLD_CFG( NAME, "gfChan4_Threshold", Units::MILLIAMPERE ); // Current above baseline that triggers a ground fault
static const ConfigURI GFCHAN5_THRESHOLD_CFG( NAME, "gfChan5_Threshold", Units::MILLIAMPERE ); // Current above baseline that triggers a ground fault

static const ConfigURI GFSCAN_TIMEOUT_CFG( NAME, "gfScanTimeout", Units::MILLIAMPERE );         // How often to scan for ground faults

}

namespace CBITMainGroundfaultIF
{
static const Str NAME( "CBITMainGroundfault" );
#include "component/HasAnalogToDigitalIF.h"
}

#endif /*CBITIF_H_*/
