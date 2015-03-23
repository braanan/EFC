/** \file
 *
 *  Contains the CBIT class implementation.
 *
 *  Copyright (c) 2007,2008,2009 MBARI
 *  MBARI Proprietary Information.  All Rights Reserved
 */

#include "CBIT.h"
#include "CBITIF.h"

#include <limits.h>
#include <stdlib.h>

#include "data/ConfigReader.h"
#include "data/Slate.h"
#include "data/UniversalDataReader.h"
#include "data/UniversalDataWriter.h"
#include "data/UniversalURI.h"
#include "supervisor/Supervisor.h"
#include "units/Units.h"
#include "sensorModule/OnboardIF.h"
#include "controlModule/SpeedControlIF.h"
#include "supervisor/CommandLine.h"

#define ATMOSPHERE_PSI (14.696 )

bool CBIT::GFScanForce_( false );

CBIT::CBIT( const Module* module )
    : SyncTestComponent( CBITIF::NAME, module ),
      wdtInitialized_( false ),
      abortDepthTimeout_( 5 ),
      envFailTimeout_( 10 ),
      battThresholdFailTimeout_( 12 ), // 3 cycles of battery data
      gfChanSetupTimeout_( 5 ),
      locationCheckTimeout_( 120 ),
      nanLocationActive_( false ),
      nanLocationReported_( false ),
      abortDepthActive_( false ),
      previouslyBelowAbort_( false ),
      stopDepthActive_( false ),
      previouslyBelowStop_( false ),
      waterDetected_( false ),
      batteryFailed_( false ),
      backplaneFailed_( false ),
      battFailCounts_( 0 ),
      battFailReport_( 10 ),
      envFailed_( false ),
      envExcursionFault_( false ),
      battThresholdFailed_( false ),
      avgHumidity_( 0 ),
      samples_( 1 ),
      cfMissing_( false ),
      gfScanInProgress_( false ),
      gfReported_( false ),
      gfScan_( OPEN_SCAN ),
      gfChan_( LPC3Reg::CHAN_5 ),
      adGFCurrent_( CBITMainGroundfaultIF::AD, CBITMainGroundfaultIF::AD_VREF, CBITMainGroundfaultIF::AD_RES, !simulateHardware(), logger_ ),
      debugEFCTimeout_(600),
      runFaultClassifier_( false ),
      runElevOffsetCalc_( false ),
      empiricalFaultCounter_( 0 ),
      empiricalFaultElevOffset_( 0 ),
      empiricalFaultBinSum_( 0 ),
      empiricalFaultEleTmp_( 0 ),
      empiricalFaultPitchTmp_( 0 ),
      empiricalFaultDepth_( 0 ),
      empiricalFaultPositiveFlagCount_( 0 )
{
    logger_.syslog( "Construct Continuous Built In Test." );

    // input readers
    clearFaultCmdReader_ = newDataReader( CBITIF::CLEAR_FAULT_CMD );
    clearLeakFaultCmdReader_ = newDataReader( CBITIF::CLEAR_LEAK_FAULT_CMD );

    // input readers from onboard sensors and other components
    pressureReader_ = newDataReader( OnboardIF::PRESSURE_READING );
    humidityReader_ = newDataReader( OnboardIF::HUMIDITY_READING );
    temperatureReader_ = newDataReader( OnboardIF::TEMPERATURE_READING );
    speedCmdReader_         = newDataReader( SpeedControlIF::SPEED_CMD );
    verticalModeReader_     = newDataReader( VerticalControlIF::VERTICAL_MODE );

    for( int i = 0; i < ( NUM_BATTSA + NUM_BATTSB ); i++ )
    {
        battPacks_[i].battTempReader_ = Slate::NewReader( Batt_Ocean_ServerIF::NAME, Batt_Ocean_ServerIF::BATT_TEMP_BASENAME + i, this, Units::CELSIUS() );
    }

    // Universal inputs
    depthReader_ = newUniversalReader( UniversalURI::DEPTH );
    depthRateReader_     = newUniversalReader( UniversalURI::DEPTH_RATE );
    elevatorAngleReader_ = newUniversalReader( UniversalURI::PLATFORM_ELEVATOR_ANGLE );
    latitudeReader_ = newUniversalReader( UniversalURI::LATITUDE );
    longitudeReader_ = newUniversalReader( UniversalURI::LONGITUDE );
    pitchReader_ = newUniversalReader( UniversalURI::PLATFORM_PITCH_ANGLE );


    // Slate outputs
    shorepowerWriter_ =  newDataWriter( CBITIF::SHOREPOWER_ON );

    faultWriter_ = newUniversalWriter( UniversalURI::PLATFORM_FAULT, Units::ENUM, 0 );
    leakFaultWriter_ = newUniversalWriter( UniversalURI::PLATFORM_FAULT_LEAK, Units::ENUM, 0 );

    gfScanActiveWriter_ = newDataWriter( CBITIF::GF_ACTIVE_STATE );

    // Ground fault current
    gfChan0CurrentWriter_ = newDataWriter( CBITIF::GFCHAN0_READING );
    gfChan1CurrentWriter_ = newDataWriter( CBITIF::GFCHAN1_READING );
    gfChan2CurrentWriter_ = newDataWriter( CBITIF::GFCHAN2_READING );
    gfChan4CurrentWriter_ = newDataWriter( CBITIF::GFCHAN4_READING );
    gfChan5CurrentWriter_ = newDataWriter( CBITIF::GFCHAN5_READING );
    gfChanOpenCurrentWriter_ = newDataWriter( CBITIF::GFCHANOPEN_READING );

    // Emperical Fault Classifier writers
    empericalFaultDetectedWriter_ = newDataWriter( CBITIF::EMPERICAL_FAULT_READING );
    binnedDepthRateWriter_ = newDataWriter( CBITIF::BINNED_DEPTH_RATE_READING );

    /// Configuration Readers
    abortDepthCfgReader_ = newConfigReader( CBITIF::ABORT_DEPTH_CFG );                // Depth at which we drop the weight. Should be greater than all depth envelopes
    stopDepthCfgReader_ = newConfigReader( CBITIF::STOP_DEPTH_CFG );                  // Depth at which we stop the mission. Should be greater than all depth envelopes and less than abort depth
    humidityThresholdCfgReader_ = newConfigReader( CBITIF::HUMIDITY_THRESHOLD_CFG );  // relative humidity
    pressureThresholdCfgReader_ = newConfigReader( CBITIF::PRESSURE_THRESHOLD_CFG );  // Onboard pressure must measure greater than this offset from 1 ATM
    tempThresholdCfgReader_ = newConfigReader( CBITIF::TEMP_THRESHOLD_CFG );          // Only need to use kelvin for temperature deltas
    vehicleOpenCfgReader_ = newConfigReader( CBITIF::VEHICLE_OPEN_CFG );
    abortDepthTimeoutCfgReader_ = newConfigReader( CBITIF::ABORT_DEPTH_TIMEOUT_CFG ); // If this time has elapsed and we're still below abort depth, drop the weight, end the mission.
    battFailReportCfgReader_ = newConfigReader( CBITIF::BATT_FAIL_REPORT_CFG );       // Report main battery failover at this multiple
    envTimeoutCfgReader_ = newConfigReader( CBITIF::ENV_TIMEOUT_CFG );                // If this time has elapsed and pressure or humidity are still above threshold, end the mission.
    faultClassifierCfgReader_ = newConfigReader( CBITIF::RUN_FAULT_CLASSIFIER );      // Runs the empirical fault classifier as part of CBIT if true
    elevOffsetCalcCfgReader_ = newConfigReader( CBITIF::RUN_ELEV_OFFSET_CALCULATOR );// Runs the empirical fault classifier's elevator offset calculator as part of CBIT if true

    battTempThresholdCfgReader_ = newConfigReader( CBITIF::BATT_TEMP_THRESHOLD_CFG ); // The allowable threshold for battery temperature
    gfChan0ThresholdCfgReader_ = newConfigReader( CBITIF::GFCHAN0_THRESHOLD_CFG );   // Current above baseline that triggers a ground fault
    gfChan1ThresholdCfgReader_ = newConfigReader( CBITIF::GFCHAN1_THRESHOLD_CFG );   // Current above baseline that triggers a ground fault
    gfChan2ThresholdCfgReader_ = newConfigReader( CBITIF::GFCHAN2_THRESHOLD_CFG );   // Current above baseline that triggers a ground fault
    gfChan4ThresholdCfgReader_ = newConfigReader( CBITIF::GFCHAN4_THRESHOLD_CFG );   // Current above baseline that triggers a ground fault
    gfChan5ThresholdCfgReader_ = newConfigReader( CBITIF::GFCHAN5_THRESHOLD_CFG );   // Current above baseline that triggers a ground fault
    gfScanTimeoutCfgReader_ = newConfigReader( CBITIF::GFSCAN_TIMEOUT_CFG ) ;         // How often to scan for ground faults

}


CBIT::~CBIT()
{}


// Initialize function
void CBIT::initialize( void )
{
    logger_.syslog( "Initialize CBIT Component." );

    this->setAllowableFailures( 1 );

    abortStartTime_ = Timestamp::Now(); // Restart the clocks
    stopDepthStartTime_ = Timestamp::Now();
    envFailStartTime_ = Timestamp::Now();
    gfStartTime_ = Timestamp::Now();
    battThresholdFailStartTime_ = Timestamp::Now();
    debugEFCTimestamp_ = Timestamp::Now();
    
    samples_ = 1;

    // Get the size of the list of components
    unsigned int listsize = ComponentRegistry::GetEntryCount();
    if( listsize <= 1 )
    {
        faultWriter_->write( Units::ENUM, true );
        logger_.syslog( Str( "CBIT failed to initialize." ), Syslog::ERROR );
    }

    if( Supervisor::WasRunning() )
    {
        logger_.syslog( "LAST RESTART WAS UNINTENTIONAL.", Syslog::FAULT );
    }

    // Initialize the watchdog timer
    // Set up to be driven by PERIPH_CLK @ 13 Mhz.
    // Full 32 bit register cycle will be 4294967295 * (1/13,000,000) ~ 5.506 minutes
    if( LPC3Reg::InitializeWDT() )
    {
        wdtInitialized_ = true;
        // Check for previous WDT reset
        if( !simulateHardware() && ( LPC3Reg::WDTimRes_ & 0x1 ) )
        {
            logger_.syslog( Str( "LAST REBOOT DUE TO WATCHDOG TIMER RESET." ), Syslog::CRITICAL );
        }
        else
        {
            logger_.syslog( Str( "Last reboot was NOT due to watchdog timer." ), Syslog::INFO );
        }
    }
    else
    {
        logger_.syslog( Str( "Watchdog Timer failed to initialize." ), Syslog::CRITICAL );
        this->setFailure( FailureMode::HARDWARE );
    }

    if( simulateHardware() )
    {
        return;
    }

    // Kick off the heartbeat
    LPC3Reg::InitializeHBC();

    // "Safe" the drop weight
    LPC3Reg::DeactivateBurnwire();

    // "Safe" the radio signals
    LPC3Reg::DeactivateRadio();
}


// Load parameters
bool CBIT::readConfig( void )
{
    // Check if all the parameters are read correctly
    bool ok = true;
    ok &= abortDepthCfgReader_->read( Units::METER, abortDepth_ );
    ok &= stopDepthCfgReader_->read( Units::METER, stopDepth_ );
    ok &= humidityThresholdCfgReader_->read( Units::PERCENT, bitHumidityThreshold_ );
    ok &= pressureThresholdCfgReader_->read( Units::POUND_PER_SQUARE_INCH, bitPressureThreshold_ );
    ok &= tempThresholdCfgReader_->read( Units::CELSIUS, bitTempThreshold_ );
    ok &= battTempThresholdCfgReader_->read( Units::CELSIUS, battTempThreshold_ );

    int boolTemp;
    ok &= vehicleOpenCfgReader_->read( Units::BOOL, boolTemp );
    cbitVehicleOpen_ = boolTemp;

    ok &= faultClassifierCfgReader_->read( Units::BOOL, boolTemp );
    runFaultClassifier_ = boolTemp;
    ok &= elevOffsetCalcCfgReader_->read( Units::BOOL, boolTemp );
    runElevOffsetCalc_ = boolTemp;

    double timeoutTemp;
    ok &= abortDepthTimeoutCfgReader_->read( Units::SECOND, timeoutTemp );
    abortDepthTimeout_ = timeoutTemp;

    int battFailReport;
    ok &= battFailReportCfgReader_->read( Units::COUNT, battFailReport );
    battFailReport_ = battFailReport;

    double timeoutEnv;
    ok &= envTimeoutCfgReader_->read( Units::SECOND, timeoutEnv );
    envFailTimeout_ = timeoutEnv;

    double timeoutScan;
    ok &= gfScanTimeoutCfgReader_->read( Units::SECOND, timeoutScan );
    gfScanTimeout_ = timeoutScan;

    // gfChanX_ThresholdCfgReader's read only during scanGFChan

    return ok;
}

/// The actual "payload" of the component
void CBIT::run()
{

    // Make sure we've got the latest from the top...
    readConfig();

    // Kick the WDT and heartbeat once per cycle
    assertVitals();

    // If Logger has received a Critical error, stop the mission
    checkCriticals();

    // Look for stop or abort depths
    checkDepth();

    // Logs the shore power status
    monitorShorePower();

    // Check for failed main battery
    checkMainBattery();

    // Check for backplane power
    checkBackplanePower();

    // Read water sensors
    checkLeak();

    // See if card is still in place
    monitorCFCard();

    // Make sure location is a real value
    //checkLocation();

    monitorEnvironmentals();

    monitorBatteryTemps();

    handleComponentFaults();

    // Check GF if it's time
    scanGFChan();

    if( runFaultClassifier_ )
    {
        empiricalFaultClassification();
    }

}

/// Uninit function
void CBIT::uninitialize( void )
{
    logger_.syslog( "Uninitialize CBIT Component." );

    if( simulateHardware() )
    {
        return;
    }

    // Power off loads
    if( !LPC3Reg::PowerOffLoads() )
    {
        logger_.syslog( Str( "Backplane failed to power down" ), Syslog::CRITICAL );
    }

    //Disable WDT.
    LPC3Reg::UninitializeWDT();

    // Open all GF detection circuits
    LPC3Reg::DeactivateGF( LPC3Reg::CHAN_0 );
    LPC3Reg::DeactivateGF( LPC3Reg::CHAN_1 );
    LPC3Reg::DeactivateGF( LPC3Reg::CHAN_2 );
    LPC3Reg::DeactivateGF( LPC3Reg::CHAN_4 );
    LPC3Reg::DeactivateGF( LPC3Reg::CHAN_5 );
}

/// Should return [myNamespace]::SIMULATE_HARDWARE, or [myNamespace]::POWER, etc
ConfigURI CBIT::getConfigURI( ConfigOption configOption ) const
{
    return configOption == CONFIG_SIMULATE_HARDWARE ? CBITIF::SIMULATE_HARDWARE : ConfigURI::NO_CONFIG_URI;
}


void CBIT::assertVitals( void )
{
    // Kick the WDT once per cycle
    if( !simulateHardware() )
    {
        LPC3Reg::StrobeHBC();
        if( wdtInitialized_ )
        {
            LPC3Reg::StrobeWDT();
        }
    }

    // Send heartbeat
    if( !simulateHardware() )
    {
        LPC3Reg::StrobeHBC();
    }
}


void CBIT::checkBackplanePower( void )
{
    // Check for backplane power
    if( !simulateHardware() && !LPC3Reg::QueryBackplanePower() )
    {
        if( backplaneFailed_ == false )
        {
            logger_.syslog( Str( "Backplane power off. Attempting to restore." ), Syslog::FAULT );
            if( LPC3Reg::PowerOnLoads() )
            {
                logger_.syslog( Str( "Backplane power restored." ), Syslog::IMPORTANT );
            }
            else
            {
                logger_.syslog( Str( "Backplane power failed to restore. Resetting CPU." ), Syslog::CRITICAL );
                backplaneFailed_ = true;
            }
        }
        else // The backplane has failed and the failure has been logged.
        {
            backplaneFailed_ = false; // Just in case...
            LPC3Reg::SWReset(); // Attempt to turn on the radio(s) via emergency card if this keeps up.
        }
    }
    else
    {
        backplaneFailed_ = false;
    }
}


void CBIT::checkCriticals( void )
{
    if( Logger::LastCriticalErrorReceived() != Timestamp::NOT_SET_TIME )
    {
        if( Supervisor::Instance_ != NULL )
        {
            CommandLine::DoSchedulePause( Syslog::FAULT, Logger::LastCriticalErrorReceived() );
            Supervisor::Instance_->stopMission();
            Logger::ClearCrtiticalErrorReceived();
        }
    }
}


void CBIT::checkDepth( void )
{
    double depth( 0 );
    if( depthReader_->isActive() && depthReader_->wasTouchedSinceLastRun( this ) && depthReader_->read( Units::METER, depth ) )
    {
        if( depth >= abortDepth_ ) // Make sure we're not below abort depth
        {
            if( ( previouslyBelowAbort_ ) && ( abortStartTime_.elapsed() > abortDepthTimeout_ ) ) // Two cycles in a row AND threshold time elapsed will result in drop weight activation
            {
                if( abortDepthActive_ == false )
                {
                    abortDepthActive_ = true;
                    logger_.syslog( "ABORT DEPTH REACHED. ACTIVATING BURNWIRE.", Syslog::CRITICAL );
                    LPC3Reg::ActivateBurnwire();
                    if( Supervisor::Instance_ != NULL ) Supervisor::Instance_->stopMission(); // stop the current mission
                }
            }
            else
            {
                previouslyBelowAbort_ = true;
            }
        }
        else
        {
            abortStartTime_ = Timestamp::Now(); // Restart the clock
            abortDepthActive_ = false;
            previouslyBelowAbort_ = false;
        }

        if( depth >= stopDepth_ ) // Check for stop depth
        {
            if( ( previouslyBelowStop_ ) && ( stopDepthStartTime_.elapsed() > abortDepthTimeout_ ) ) // Two cycles in a row AND threshold time elapsed will result in mission termination
            {
                if( stopDepthActive_ == false )
                {
                    stopDepthActive_ = true;
                    logger_.syslog( "STOP DEPTH REACHED. Terminating Mission.", Syslog::CRITICAL );
                    if( Supervisor::Instance_ != NULL ) Supervisor::Instance_->stopMission(); // stop the current mission
                }
            }
            else
            {
                previouslyBelowStop_ = true;
            }
        }
        else
        {
            stopDepthStartTime_ = Timestamp::Now(); // Restart the clock
            stopDepthActive_ = false;
            previouslyBelowStop_ = false;
        }
    } // if you can read valid depth

    // TODO: Implement a new feature in CBIT that warns if depth has not been written recently.
}


void CBIT::checkLeak( void )
{
    // Check for water in the hull
    if( !simulateHardware() )
    {
        if( LPC3Reg::WaterDetected() && ( !waterDetected_ ) )
        {
            waterDetected_ = true;
            LPC3Reg::ActivateBurnwire();
            logger_.syslog( Str( "WATER DETECTED IN PRESSURE HULL. BURNWIRE ACTIVATED" ), Syslog::CRITICAL );
            // Stop the current mission
            if( Supervisor::Instance_ != NULL )
            {
                Supervisor::Instance_->stopMission();
            }
        }
    }
}


void CBIT::checkLocation( void )
{
    double latitude = nanf( "" );
    double longitude = nanf( "" );
    latitudeReader_->read( Units::RADIAN, latitude );
    longitudeReader_->read( Units::RADIAN, longitude );

    if( isnan( latitude ) || isnan( longitude ) )
    {
        // Start the clock if we haven't yet
        if( !nanLocationActive_ )
        {
            locationCheckStartTime_ = Timestamp::Now();
            nanLocationActive_ = true; // We have encountered a nan location
            logger_.syslog( "NAN location detected.", Syslog::IMPORTANT );
            // TODO: Is there a good way to determine which component wrote it?
        }

        if( locationCheckStartTime_.elapsed() > locationCheckTimeout_ && !nanLocationReported_ )
        {
            logger_.syslog( Str( "Location has been nan for set timeout." ), Syslog::CRITICAL );
            nanLocationReported_ = true;
        }
    }
    else
    {
        nanLocationActive_ = false;
        nanLocationReported_ = false;
    }
}


void CBIT::checkMainBattery( void )
{
    //Check for failed main battery
    if( !simulateHardware() && !LPC3Reg::QueryBattPower() )
    {
        if( batteryFailed_ == false )
        {
            if( ( battFailCounts_ % battFailReport_ ) == 0 )
            {
                battFailCounts_ += 1;
                logger_.syslog( Str( "Main Battery Failure. Count: " + Str( battFailCounts_ ) ), Syslog::FAULT );
            }
            else
            {
                battFailCounts_ += 1;
            }
            batteryFailed_ = true;
        }
    }
    else
    {
        batteryFailed_ = false;
    }
}


void CBIT::empiricalFaultClassification( void )
{
    const double pi = 3.1415926;
    double depth( 0 );
    float  pitch( nanf( "" ) );
    float  depthRate( nanf( "" ) );
    float  elevAngle( nanf( "" ) );
    float  speedCmd( nanf( "" ) );
    int    verticalMode = -1;
    int    EFCbinSize = 8;


    // Check incoming data quality
    bool dataCheckEFC[7] = { false };
    if( speedCmdReader_->isActive() && speedCmdReader_->read( Units::METER_PER_SECOND, speedCmd ) )
    { dataCheckEFC [0] = true; }
    if( depthReader_->isActive() && depthReader_->read( Units::METER, depth ) )
    { dataCheckEFC [1] = true; }
    if( depthRateReader_->isActive() && depthRateReader_->read( Units::METER_PER_SECOND, depthRate ) )
    { dataCheckEFC [2] = true; }
    if( elevatorAngleReader_->isActive() && elevatorAngleReader_->read( Units::RADIAN, elevAngle ) )
    { dataCheckEFC [3] = true; }
    if( pitchReader_->isActive() && pitchReader_->read( Units::RADIAN, pitch ) )
    { dataCheckEFC [4] = true; }
    if( verticalModeReader_->isActive() && verticalModeReader_->read( Units::ENUM, verticalMode ) )
    { dataCheckEFC [5] = true; }
    if( depth > 1 && speedCmd == 1 && verticalMode == 5 && !isnan( depthRate ) && !isnan( elevAngle ) && !isnan( pitch ) )
    { dataCheckEFC [6] = true; }    // Make sure operational profile is compatible for analysis
    

    // Incoming data check-sum
    int dataChecksumEFC = 0;
    for( int k = 0; k < 7; k++ )
    {
        dataChecksumEFC += dataCheckEFC[k];
    }

    // Get to work if data checks out
    if( dataChecksumEFC == 7 )
    {
        

        // Elevator offset calculator
        if( runElevOffsetCalc_ )
        {

            empiricalFaultCounter_ += 1;           // Pulse counter
            empiricalFaultEleTmp_  += elevAngle;  // Sum-up elev angle for bin avg.

            
            int EOCsampleSize[5] = {1,900,1800,3600,7200};
            for( int k=0; k<5; k++ )
            {
                if (empiricalFaultCounter_ == EOCsampleSize[k])
                {
                    empiricalFaultElevOffset_ = empiricalFaultEleTmp_ / EOCsampleSize[k];  // Calc mean elevator angle (offset)
                    
                    // Write offset value to syslog
                    logger_.syslog( "EFC: Completed elev offset calculation (n=" + Str( EOCsampleSize[k] ) + ") - ElevOffset = "
                                   + Str( empiricalFaultElevOffset_ ), Syslog::IMPORTANT );

                    if(empiricalFaultCounter_ == EOCsampleSize[4])
                    {
                        runElevOffsetCalc_ = false; // Disable Elev offset calculator and turn on Fault Classification
                        CommandLine::DoCommand( "configSet runElevOffsetCalc 0 bool persist", false ); // Enforce via command line
                        empiricalFaultCounter_ = 0;      // Reset counter
                        empiricalFaultEleTmp_ = 0;      // Reset Elev sum-up
                        
                    }
                    
                    break;
                }
            }
        }
        
        
        
        // EFC
        if( !runElevOffsetCalc_ )
        {
            
            empiricalFaultBinSum_  += depthRate; // Sum-up depthRate for bin avg.
            empiricalFaultCounter_ += 1;        // Pulse counter
            
            // Grab timestamp and depth when bin is half full
            if( empiricalFaultCounter_ == round( EFCbinSize/2 ) )
            {
                empiricalFaultTimestamp_ = Timestamp::Now();
                empiricalFaultDepth_     = depth;
            }
            
            // Get extream pitch angle value for bin
            if( fabs( empiricalFaultPitchTmp_ ) < fabs( pitch ) )
            {
                empiricalFaultPitchTmp_ = pitch;
            }

            // Get extream elevator angle value for bin
            elevAngle = elevAngle - 0.85 * empiricalFaultElevOffset_; // Correct elevator angle offset
            if( fabs( empiricalFaultEleTmp_ ) < fabs( elevAngle ) )
            {
                empiricalFaultEleTmp_ = elevAngle;
            }
            

            // Compute bin avg and evaluate filtering criteria if bin size is reached
            if( empiricalFaultCounter_ == EFCbinSize )
            {
                float binAvgDepthRate = ( empiricalFaultBinSum_ / EFCbinSize );

                // Empirical fault classification filtering criteria (return non-zero if anomaly detected)
                int qual = 0;
                if( empiricalFaultEleTmp_ < ( -10 * ( pi / 180 ) ) && ( binAvgDepthRate <  0.1 ) ) // Commanding extream Ele up in non-transition state
                { qual = 1; }
                if( empiricalFaultEleTmp_ > ( 10 * ( pi / 180 ) ) && ( binAvgDepthRate > -0.1 ) )  // Commanding extream Ele down in non-transition state
                { qual = 2; }
                if( empiricalFaultEleTmp_ < ( 0 ) && ( binAvgDepthRate > 0.4 ) )                   // Going down fast when Ele commanding up
                { qual = 3; }
                if( empiricalFaultEleTmp_ < ( 0 ) && ( empiricalFaultPitchTmp_ < ( -30 * ( pi / 180 ) ) ) ) // Pitched down hard when Ele commanding up
                { qual =4; }
                if( empiricalFaultPitchTmp_ < ( -35 * ( pi / 180 ) ) )                               // Pitched down very hard (case control-loop fails).
                { qual = 5; }
                
                
                if( (qual!=0) & (empiricalFaultDepth_>2) )
                {
                    // Count consecituve fault classifications
                    empiricalFaultPositiveFlagCount_ += 1;
                    
                    if( empiricalFaultPositiveFlagCount_ > 2 ) // arm. safe. fire. (Fault persists)
                    {
                        // Write to syslog
                        logger_.syslog( "empiricalFaultClassification: Fault classified (type " + Str( qual ) +
                                    ") - Depth = " + Str( empiricalFaultDepth_ ) +
                                    ", binAvgDepthRate =  " + Str( binAvgDepthRate ) +
                                    ", pitchAngle = " + Str( empiricalFaultPitchTmp_ * ( 180 / pi ) ) +
                                    ", elevAngle = " + Str( empiricalFaultEleTmp_ * ( 180 / pi ) ), Syslog::INFO );
                    }
                }
                else { empiricalFaultPositiveFlagCount_ = 0; } // Reset positive flag Count sequence case no fault
                

                // Write data
                empericalFaultDetectedWriter_->write( Units::ENUM, qual, empiricalFaultTimestamp_ );
                binnedDepthRateWriter_->write( Units::METER_PER_SECOND, binAvgDepthRate, empiricalFaultTimestamp_ );

                // Reset counter / binSum / eleTmp / pitchTmp
                empiricalFaultCounter_   = 0;
                empiricalFaultBinSum_    = 0;
                empiricalFaultEleTmp_    = 0;
                empiricalFaultPitchTmp_  = 0;
                empiricalFaultTimestamp_ = 0;
                empiricalFaultDepth_     = 0;
                
                if( debugEFCTimestamp_.elapsed() > debugEFCTimeout_ )
                {
                    logger_.syslog( "empiricalFaultClassification: EFC running - evaluating filtering criteria", Syslog::DEBUG);
                    debugEFCTimestamp_ = Timestamp::Now();
                }
            }
        }
    }

    // Data check-sum false - reset counter / binsum / eletmp / pitchtmp
    else
    {
        if( !runElevOffsetCalc_ )
        {
            empiricalFaultCounter_  = 0;
            empiricalFaultEleTmp_   = 0;
        }
        empiricalFaultBinSum_    = 0;
        empiricalFaultPitchTmp_  = 0;
        empiricalFaultTimestamp_ = 0;
        empiricalFaultDepth_     = 0;
        
        if( debugEFCTimestamp_.elapsed() > debugEFCTimeout_ )
        {
            logger_.syslog( "empiricalFaultClassification: EFC running - data check-sum false", Syslog::DEBUG);
            debugEFCTimestamp_ = Timestamp::Now();
        }
    }
}


void CBIT::handleComponentFaults( void )
{
    // Check for reported faults
    if( clearFaultCmdReader_->isActive()
            && clearFaultCmdReader_->wasTouchedSinceLastRun( this )
            && clearFaultCmdReader_->asInt( Units::ENUM ) != 0 )
    {
        faultWriter_->write( Units::ENUM, false );
    }
    if( clearLeakFaultCmdReader_->isActive()
            && clearLeakFaultCmdReader_->wasTouchedSinceLastRun( this )
            && clearLeakFaultCmdReader_->asInt( Units::ENUM ) != 0 )
    {
        faultWriter_->write( Units::ENUM, false );
    }

    // Now the code that sets the faults to positive follows:
    unsigned int listsize = ComponentRegistry::GetEntryCount();

    // Get a list of all the components
    for( unsigned int i = 0; i < listsize; i++ )
    {
        Component* component = ( Component* )ComponentRegistry::GetIndexed( i );
        // First check for null
        if( component != NULL )
        {
            // Then see if there is a failure
            if( ( component->isFailed() ) )
            {
                // Report as an error for the syslog
                if( !( component->getFailureReported() ) )
                {
                    logger_.syslog( FailureMode::ToString( component->getFailureType() ) + " in component: " + component->getName(), Syslog::ERROR );
                    component->setFailureReported( true );
                }
                // Check if there have been multiple failures
                if( component->getFailCount() >= component->getAllowableFailures() )
                {
                    faultWriter_->write( Units::ENUM, true );
                    // Set this component as reported so we don't keep sending info
                    if( !( component->getCriticalFailureReported() ) )
                    {
                        logger_.syslog( FailureMode::ToString( component->getFailureType() ) + " in component: " + component->getName(), component->isFailureMissionCritical() ? Syslog::CRITICAL : Syslog::FAULT );
                        component->setCriticalFailureReported( true );
                    }
                    // Check to see if it's time to retry this component
                    if( component->getTimeOfFailure().elapsed() > component->getRetryTimeout() && component->isFailureUninitialized() )
                    {
                        logger_.syslog( "Clearing failed count for component " + component->getName(), Syslog::INFO );
                        component->setFailure( FailureMode::NONE );
                        component->resetFailCount();
                        component->setCriticalFailureReported( false );
                    }
                }
                // Check to see if it's time to retry this component
                else if( component->isFailureUninitialized() )
                {
                    logger_.syslog( "Clearing failed state for component " + component->getName(), Syslog::INFO );
                    component->setFailure( FailureMode::NONE );
                }
            }
            // Clear our failure reporting bool
            else if( !( component->isFailed() ) && ( component->getFailureReported() ) )
            {
                component->setFailureReported( false );
                component->setCriticalFailureReported( false );
            }
        }

    }
}


void CBIT::monitorBatteryTemps( void )
{
    // Check battery thresholds not checked in battery component
    int temp;
    for( int i = 0; i < ( NUM_BATTSA + NUM_BATTSB ); i++ )
    {
        if( battPacks_[i].battTempReader_->isActive() && battPacks_[i].battTempReader_->wasTouchedSinceLastRun( this ) )
        {
            bool noTempFault = true;
            battPacks_[i].battTempReader_->read( Units::CELSIUS, temp );

            if( temp > battTempThreshold_ )
            {
                noTempFault = false;
                if( ( battThresholdFailed_ == false ) && ( battThresholdFailStartTime_.elapsed() > battThresholdFailTimeout_ ) )
                {
                    logger_.syslog( Str( "Battery temperature above threshold." ), Syslog::CRITICAL );
                    battThresholdFailed_ = true;
                    for( int j = 0; j < ( NUM_BATTSA + NUM_BATTSB ); j++ )
                    {
                        int temp;
                        battPacks_[j].battTempReader_->read( Units::CELSIUS, temp );
                        logger_.syslog( Str( "Batt #" + Str( j ) + " " + Str( temp ) + " deg C." ), Syslog::IMPORTANT );
                    }
                }
            }

            if( noTempFault )  // Nothing out of range this time around
            {
                battThresholdFailStartTime_ = Timestamp::Now();
                battThresholdFailed_ = false;
            }

        }
    }
}


void CBIT::monitorCFCard( void )
{
    // Monitor CF card
    if( !simulateHardware() && LPC3Reg::CFMissing() )
    {
        if( !cfMissing_ )
        {
            logger_.syslog( Str( "MMC card no longer detected\n" ), Syslog::CRITICAL );
            cfMissing_ = true;
        }
    }
    else
    {
        cfMissing_ = false;
    }
}


void CBIT::monitorEnvironmentals( void )
{
    // Check environmental thresholds
    if( !cbitVehicleOpen_ )
    {
        double pressure;
        int humidity, temperature;
        pressureReader_->read( Units::POUND_PER_SQUARE_INCH, pressure );
        humidityReader_->read( Units::PERCENT, humidity );
        temperatureReader_->read( Units::CELSIUS, temperature );

        if( ( fabs( pressure - ATMOSPHERE_PSI ) < bitPressureThreshold_ ) || ( humidity > bitHumidityThreshold_ ) || ( temperature > bitTempThreshold_ ) )
        {
            if( ( envFailed_ == false ) && ( envFailStartTime_.elapsed() > envFailTimeout_ ) )
            {
                logger_.syslog( Str( "Environmental Failure. Press:" + Str( pressure ) + " PSI. " + "Humidity:" + Str( humidity ) + "%. Temp:" + Str( temperature ) + " C. ABORTING MISSION" ), Syslog::CRITICAL );
                envFailed_ = true;
                // Stop the current mission
                if( Supervisor::Instance_ != NULL )
                {
                    Supervisor::Instance_->stopMission();
                }
            }
        }
        else
        {
            envFailStartTime_ = Timestamp::Now();
            envFailed_ = false;
        }

        if( samples_ < 100 )
        {
            avgHumidity_ += humidity;
            if( ++samples_ == 100 )
            {
                avgHumidity_ = avgHumidity_ / samples_;
            }
        }
        // Check to see if environmental data has jumped outside of 50% of baseline
        // 10% is the low end of the instrument measurement. Don't bother checking the delta if it's less than 10%RH
        else if( ( humidity >= 10 ) && ( humidity > avgHumidity_ * 1.5 ) )
        {
            if( envExcursionFault_ == false )
            {
                logger_.syslog( Str( "Humidity exceeds 50% of running average: Humidity:" + Str( humidity ) + " %. Average:" + Str( avgHumidity_ ) ), Syslog::FAULT );
                envExcursionFault_ = true;
            }
        }
    }
}


void CBIT::monitorShorePower( void )
{
    // Monitor shore power
    if( simulateHardware() || LPC3Reg::QueryShorepower() )
    {
        shorepowerWriter_->write( Units::BOOL, true );
    }
    else
    {
        shorepowerWriter_->write( Units::BOOL, false );
    }
}


void CBIT::scanGFChan( void )
{
    if( simulateHardware() )
    {
        return;
    }


    if( ( gfStartTime_.elapsed() < gfScanTimeout_ ) && !GFScanForce_ && !gfScanInProgress_ && !gfScanActiveWriter_->isDataRequested() )
    {
        return;
    }

    bool ok( true );
    ok &= gfChan0ThresholdCfgReader_->read( Units::MILLIAMPERE, gfChan0Threshold_ );
    ok &= gfChan1ThresholdCfgReader_->read( Units::MILLIAMPERE, gfChan1Threshold_ );
    ok &= gfChan2ThresholdCfgReader_->read( Units::MILLIAMPERE, gfChan2Threshold_ );
    ok &= gfChan4ThresholdCfgReader_->read( Units::MILLIAMPERE, gfChan4Threshold_ );
    ok &= gfChan5ThresholdCfgReader_->read( Units::MILLIAMPERE, gfChan5Threshold_ );


    if( !gfScanInProgress_ )
    {
        logger_.syslog( "Beginning GF scan", Syslog::IMPORTANT );

        // Those negatively affected by a scan are being warned
        gfScanActiveWriter_->write( Units::BOOL, true );
        gfScanInProgress_ = true;
        gfScan_ = OPEN_SCAN;
    }

    switch( gfScan_ )
    {
    case OPEN_SCAN:
        // Read open circuit value for baseline
        gfCurrentOpen_ = ( adGFCurrent_.readVolts() / ( 20 * 402 ) ) * 1000 ; // Gain is 20 and sense resistor is 402. Convert to mA
        gfScan_ = CHAN_5_SCAN;
        gfChan_ = LPC3Reg::CHAN_5;
        gfStartTime_ = Timestamp::Now();
        break;
    case CHAN_5_SCAN:
        LPC3Reg::ActivateGF( gfChan_ );
        if( gfStartTime_.elapsed() >= gfChanSetupTimeout_ )
        {
            gfCurrent5_ = ( adGFCurrent_.readVolts() / ( 20 * 402 ) ) * 1000 ; // Gain is 20 and sense resistor is 402. Convert to mA

            LPC3Reg::DeactivateGF( gfChan_ );
            gfScan_ = CHAN_4_SCAN;
            gfChan_ = LPC3Reg::CHAN_4;
            gfStartTime_ = Timestamp::Now();
        }
        break;
    case CHAN_4_SCAN:
        LPC3Reg::ActivateGF( gfChan_ );
        if( gfStartTime_.elapsed() >= gfChanSetupTimeout_ )
        {
            gfCurrent4_ = ( adGFCurrent_.readVolts() / ( 20 * 402 ) ) * 1000 ; // Gain is 20 and sense resistor is 402. Convert to mA

            LPC3Reg::DeactivateGF( gfChan_ );
            gfScan_ = CHAN_2_SCAN;
            gfChan_ = LPC3Reg::CHAN_2;
            gfStartTime_ = Timestamp::Now();
        }
        break;
    case CHAN_3_SCAN: // Not used
        break;
    case CHAN_2_SCAN:
        LPC3Reg::ActivateGF( gfChan_ );
        if( gfStartTime_.elapsed() >= gfChanSetupTimeout_ )
        {
            gfCurrent2_ = ( adGFCurrent_.readVolts() / ( 20 * 402 ) ) * 1000 ; // Gain is 20 and sense resistor is 402. Convert to mA

            LPC3Reg::DeactivateGF( gfChan_ );
            gfScan_ = CHAN_1_SCAN;
            gfChan_ = LPC3Reg::CHAN_1;
            gfStartTime_ = Timestamp::Now();
        }
        break;
    case CHAN_1_SCAN:
        LPC3Reg::ActivateGF( gfChan_ );
        if( gfStartTime_.elapsed() >= gfChanSetupTimeout_ )
        {
            gfCurrent1_ = ( adGFCurrent_.readVolts() / ( 20 * 402 ) ) * 1000 ; // Gain is 20 and sense resistor is 402. Convert to mA

            LPC3Reg::DeactivateGF( gfChan_ );
            gfScan_ = CHAN_0_SCAN;
            gfChan_ = LPC3Reg::CHAN_0;
            gfStartTime_ = Timestamp::Now();
        }
        break;
    case CHAN_0_SCAN:
        LPC3Reg::ActivateGF( gfChan_ );
        if( gfStartTime_.elapsed() >= gfChanSetupTimeout_ )
        {
            gfCurrent0_ = ( adGFCurrent_.readVolts() / ( 20 * 402 ) ) * 1000 ; // Gain is 20 and sense resistor is 402. Convert to mA

            LPC3Reg::DeactivateGF( gfChan_ );
            gfScan_ = CALCULATE;
        }
        break;
    case CALCULATE:
        // Write all the values
        gfChan0CurrentWriter_->write( Units::MILLIAMPERE, gfCurrent0_ );
        gfChan1CurrentWriter_->write( Units::MILLIAMPERE, gfCurrent1_ );
        gfChan2CurrentWriter_->write( Units::MILLIAMPERE, gfCurrent2_ );
        gfChan4CurrentWriter_->write( Units::MILLIAMPERE, gfCurrent4_ );
        gfChan5CurrentWriter_->write( Units::MILLIAMPERE, gfCurrent5_ );
        gfChanOpenCurrentWriter_->write( Units::MILLIAMPERE, gfCurrentOpen_ );

        int totalFaults = 0;

        // Chan 5
        //if (( gfCurrent5_ - gfCurrentOpen_ ) > gfChan5Threshold_ )
        if( gfCurrent5_ > gfChan5Threshold_ )  // Currently using 0 as baseline since current amplifier can vary greatly when reading open circuit
        {
            totalFaults += 1;
        }

        // Chan 4
        //if (( gfCurrent4_ - gfCurrentOpen_ ) > gfChan4Threshold_ )
        if( gfCurrent4_  > gfChan4Threshold_ )  // Currently using 0 as baseline since current amplifier can vary greatly when reading open circuit
        {
            totalFaults += 1;
            if( totalFaults != 2 )
            {
                reportGFFault( "GF scan fault mismatch - Fault on ch. 4 but not on ch. 5" );
            }
        }
        else if( totalFaults == 1 )
        {
            reportGFFault( "Chan 4 High side GF detected" );
        }

        // Chan 2 (3 not used)
        //if (( gfCurrent2_ - gfCurrentOpen_ ) > gfChan2Threshold_ )
        if( gfCurrent2_  > gfChan2Threshold_ )  // Currently using 0 as baseline since current amplifier can vary greatly when reading open circuit
        {
            totalFaults += 1;
        }
        else if( totalFaults == 2 )
        {
            reportGFFault( "Chan 2 High side GF detected" );
        }

        // Chan 1
        //if (( gfCurrent1_ - gfCurrentOpen_ ) > gfChan1Threshold_ )
        if( gfCurrent1_  > gfChan1Threshold_ )  // Currently using 0 as baseline since current amplifier can vary greatly when reading open circuit
        {
            totalFaults += 1;
        }
        else if( totalFaults == 3 )
        {
            reportGFFault( "Chan 1 High side GF detected" );
        }

        // Chan 0
        //if (( gfCurrent0_ - gfCurrentOpen_ ) > gfChan0Threshold_ )
        if( gfCurrent0_ > gfChan0Threshold_ )  // Currently using 0 as baseline since current amplifier can vary greatly when reading open circuit
        {
            totalFaults += 1;
        }
        else if( totalFaults == 4 )
        {
            reportGFFault( "Chan 0 High side GF detected" );
        }

        if( totalFaults == 5 )
        {
            reportGFFault( "Low side GF detected" );
        }

        if( totalFaults == 0 )
        {
            if( GFScanForce_ )
            {
                logger_.syslog( "No ground fault detected", Syslog::IMPORTANT );
            }
            else
            {
                logger_.syslog( "No ground fault detected", Syslog::INFO );
            }
            gfReported_ = false;
        }

        gfScan_ = OPEN_SCAN;
        gfChan_ = LPC3Reg::CHAN_5;
        gfScanInProgress_ = false;
        GFScanForce_ = false;
        gfStartTime_ = Timestamp::Now();
        gfScanActiveWriter_->write( Units::BOOL, false );
        break;
    }
}


void CBIT::reportGFFault( Str faultTxt )
{
    if( !gfReported_ || GFScanForce_ )  // if the scan was forced, repeat error reporting
    {
        logger_.syslog( faultTxt + "\nmA:\nCHAN 5 (24V): " + gfCurrent5_ + "\nCHAN 4 (Batt): " + gfCurrent4_ + "\nCHAN 2 (12V): " + gfCurrent2_ + "\nCHAN 1 (5V): " + gfCurrent1_ + "\nCHAN 0 (3.3V): " + gfCurrent0_ + "\nOPEN: " + gfCurrentOpen_ + "\nFull Scale Calc: 0.392", Syslog::FAULT );
        gfReported_ = true;
    }
}


void CBIT::SetGFScanForce( bool forceVal )
{
    GFScanForce_ = forceVal;
}


// Initialize battery pack values
CBIT::BattPack::BattPack()
    :
    voltage_( 0.0 ),
    battVoltageReader_( NULL ),
    current_( 0.0 ),
    battCurrentReader_( NULL ),
    temp_( 0.0 ),
    battTempReader_( NULL )
{}

