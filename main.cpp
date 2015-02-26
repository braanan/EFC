//
//  main.cpp
//  EFC
//
//  Created by Ben Yair Raanan on 1/28/15.
//  Copyright (c) 2015 Ben Yair Raanan. All rights reserved.
//

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <limits.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string>



int main()
{
    
    const double pi = 3.1415926;
    int   empiricalFaultCounter_  = 0;
    double empiricalFaultBinSum_   = 0;
    double empiricalFaultEleTmp_   = 0;
    double empiricalFaultPitchTmp_ = 0;
    double empiricalFaultDepth_    = 0;
    double empiricalFaultTimestamp_= 0;
    int binSize = 8;
    int i = 0;
    
    
    
    //read in data from txt file
    std::string line;
    std::ifstream infile;  infile.open( "/Users/benya/Desktop/seed/grab.txt" );
    std::ofstream outfile; outfile.open( "/Users/benya/Desktop/seed/EFC_output.txt" );

    
    if(!infile) { //error case file can't be open
        std::cout << "Cannot open grab.txt file.\n";
        return 1;
    }
    else {
        std::cout << "Reading data from grab.txt...\n";
    }
    
    
    
    //print headers
    std::getline( infile, line );
    std::cout << "Headers: " << line << "\n\n";;
    outfile << "time depth binAvgDepthRate pitchAngle elevAngle\n"; //write headers to output file
    
    
    //read stream line by line
    while( std::getline( infile, line ) )
    {
        if( ++i > 1 ) // skip header line
        {
            //make a stream for the line itself
            std::istringstream in( line );
            
            //now read the whitespace-separated floats
            double matime, depth, depthRate, speedCmd, pitchAngle, elevAngle;
            in >> matime >> depth >> depthRate >> speedCmd >> pitchAngle >> elevAngle;

            
            //check incoming data and make sure operational profile is compatible for analysis
            bool  dataCheckEFC[6] = { false }; int dataChecksumEFC = 0;
            
            if( !isnan( speedCmd ) )            { dataCheckEFC [0] = true; }
            if( !isnan( depth ) )               { dataCheckEFC [1] = true; }
            if( !isnan( depthRate ) )           { dataCheckEFC [2] = true; }
            if( !isnan( elevAngle ) )           { dataCheckEFC [3] = true; }
            if( !isnan( pitchAngle ) )          { dataCheckEFC [4] = true; }
            if( depth > 2 && speedCmd > 0.8 )   { dataCheckEFC [5] = true; }
            
            //incoming data check-sum
            for( int k = 0; k < 6; k++ )
            {
                dataChecksumEFC += dataCheckEFC[k];
            }
            
            //get to work if data checks out
            if( dataChecksumEFC == 6 )
            {
                //std::cout << "Data check-sum true - evaluating cell " << "\n";
                
                
                //sum-up for bin avg.
                empiricalFaultBinSum_ += depthRate;
                
                
                //get extream pitch angle value for bin
                if( fabs( empiricalFaultPitchTmp_ ) < fabs( pitchAngle ) )
                {
                    empiricalFaultPitchTmp_ = pitchAngle;
                }
                
                //get extream elevator angle value for bin
                if( fabs( empiricalFaultEleTmp_ ) < fabs( elevAngle ) )
                {
                    empiricalFaultEleTmp_ = elevAngle;
                }
                
                
                
                //pulse counter
                empiricalFaultCounter_ += 1;
                
                
                
                //grab timestamp and depth when bin is half full
                if( empiricalFaultCounter_ == round( binSize/2 ) )
                {
                    //time_t  timev;
                    empiricalFaultTimestamp_ = matime; //time( &timev );
                    empiricalFaultDepth_ = depth;
                }
                
                //compute bin avg and evaluate filtering criteria if bin size is reached
                if( empiricalFaultCounter_ == binSize )
                {
                    float binAvgDepthRate = ( empiricalFaultBinSum_ / binSize );
                    //std::cout << "Bin full - Avg. depth_rate = " <<  binAvgDepthRate << "counter = " << empiricalFaultCounter_ << ", i = " << i << "\n";
                    
                    //empirical fault classification filtering criteria (return true if anomaly detected)
                    bool qual = ( ( empiricalFaultEleTmp_ < ( -10* (pi/180) ) && ( binAvgDepthRate <  0.1)) || // Commanding extreme Ele up in non-transition state. or...
                                 ( empiricalFaultEleTmp_ > (  10* (pi/180) ) && ( binAvgDepthRate > -0.1))  || // Commanding extreme Ele down in non-transition state. or...
                                 ( empiricalFaultEleTmp_ < ( 0 ) && ( binAvgDepthRate > 0.4))               || // Going down fast when Ele commanding up. or...
                                 ( empiricalFaultEleTmp_ < ( 0 ) && (empiricalFaultPitchTmp_ < ( -30* (pi/180)))) || // Pitched down hard when Ele commanding up. or...
                                 ( empiricalFaultPitchTmp_ < ( -35* (pi/180) )) );                                   // Pitched down very hard (case control-loop fails)
                    
                    //write
                    if( qual==1) {
                        //screen
                        std::cout << "\n";
                        std::cout << "qual = " << qual << " at " << empiricalFaultTimestamp_ << " counter = " << empiricalFaultCounter_ << ", i = " << i << "\n";
                        std::cout << "binAvgDepthRate = " << binAvgDepthRate << "\n";
                        std::cout << "depth = " << depth << "\n";
                        std::cout << "pitch = " << empiricalFaultPitchTmp_ * (180/pi) << "\n";
                        std::cout << "elevAngle = " << empiricalFaultEleTmp_ * (180/pi) << "\n" << "\n";
                        //.txt
                        outfile << std::fixed << std::setprecision(9) << empiricalFaultTimestamp_
                                << " " << std::setprecision(5) << depth << " " << binAvgDepthRate <<
                                " " << empiricalFaultPitchTmp_ << " " << empiricalFaultEleTmp_ << "\n";
                    }
                    
                    //reset counter / binSum / eleTmp / pitchTmp
                    empiricalFaultCounter_  = 0;
                    empiricalFaultBinSum_   = 0;
                    empiricalFaultEleTmp_   = 0;
                    empiricalFaultPitchTmp_ = 0;
                    empiricalFaultTimestamp_= 0;
                    //std::cout << "Completed bin evaluation - counter reset (i = " << i << ") " << "\n";
                }
            }
            
            //data check-sum false - reset counter / binsum / eletmp / pitchtmp
            else
            {
                empiricalFaultCounter_  = 0;
                empiricalFaultBinSum_   = 0;
                empiricalFaultEleTmp_   = 0;
                empiricalFaultPitchTmp_ = 0;
                empiricalFaultTimestamp_= 0;
                //std::cout << "Data check-sum false - resetting counter (i = " << i <<") " << "\n" << "\n";
            }
        }
    }
    
    //close the opened files
    infile.close();
    outfile.close();
    return 0;
}
