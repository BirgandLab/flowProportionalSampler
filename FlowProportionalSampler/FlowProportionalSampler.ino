///
//FLOW PROPORTIONAL SAMPLER
//will interface with Sontek to monitor flow for flow proportional sampler
//will run standard SCAAN/MANTA setup
//will use peristaltic pump and 3-way valves to clean SCAAN cuvette
//will encorporate RTC and SD cards to monitor and log activity
//might use webcam for site monitoring/miniGaugeCam incorporation.
//10-1-2013 assembled rudimentary 1st setup. 
//10-1-2013 tested system in flume improved some details and feedback
///
//TODO
//make sure that eeprom is getting written to evenly--cycle through it also make
//sure that serial number (4digit hex) is written to block 0-3 of the EEprom, 
//and not overwritten check to see if all shut offs work--bottle full, no it would
//be nice to stop things from running if something goes bad--pump blows off 
//tubing etc make sure that before the sytem runs it checks temp, nut just before 
//FP sample routine.
//2-18-2014 added DrainAllValves routine to push water out of FPS sample loop and valves

    


#include <math.h>				//library for making calculations
#include <Wire.h>               //i2c library for communication with RTC
#include <Chronodot.h>          //RealTime Clock library
#include <SD.h>                 //SD card library
#include <ModbusMaster.h>       //The MODBUS protocol for communication with Sontek
#include <EEPROM.h>             //Enable storing data in non volatile memory
#include "EEPROMAnything.h"     //Data structure to facilitate EEPROM addressing

/*libraries that have been used, might be used, but are not presently used*/
//#include <SoftwareSerial.h>      //To talk serial over digital channels with camera
//#include <LowPower.h>            //Enable sleeping etc.
//#include <Adafruit_VC0706.h>     //CVCO706 web cam library

//Version 0.1--seems to work adding structure to handle decision variables for clarity
//             needs some logic cleaned up --should NOT pump at all when temp is too low
//             version# printed in log file
//             header added to system Log File
//             spacing improved in Sontek log file
//             check battery voltage conversion
//             what else?
//             at revisions 19 and 20 in tortoise SVN repository
//             added allClear() function to make sensor related decisions--too cold 
//                   (don't run pumps, but collect data) bottle full(run pumps but hold
//					  off on sample collection)
//            corrected float error in sample # collection
//            corrected rounding problem that prevented sample collection when 
//					  TimeThreshold was met but QThreshold was not
//            
//VERSION 0.2  
//            added SD reader to read config file from SD card root directory
//            reworked Q calculation and structure
//            added ability to sample at 1  minute intervals from Sontek
//                will adjust FPS sample routine to be called from regular Sontek 
//				  queries OR from SCAN system routine
//                this means setting it aside as a routine
//			  Actually holding off on that for now. Just 15 minute schedule.
//TODO		  Add a section to all clear to check to see how much of the Sontek is 
//			  submerged. Use water level to decide whether or not to run pumps

//deployed 11-22-2013 claridge upstream.
//changed pump times from previous version.
//having problems with Sontek data.

//3-5-2014 added FPSTest and FPSTestSamples to variables read from SD card
//added A0 as groundable signal to start a sample routine
//modified Q bookkeeping to carry over flow greater than the QThreshold into the next sample interval
//moved some round statements in the FPS collection section
//

const float VersionNumber=0.3; //update version (probably good to add date too)


///Declare a MODBUS object name(address, serial port)
ModbusMaster sontek(1,1);   
     
	 
///Declare a union for converting data from the Sontek registers data to decimal values
//transfers bytes to floats, integers and chars
union u_tag {                    
    byte b[4];                   
    float fval;
    int ival;
    char cat[10];
} registerValue;


///Declare a structure to hold cumulative and discrete flow data from Sontek
//and from a rating curve/equation.
struct config_t
{
    float  QCmlSontek;							//cumulative flow in Sontek memory
    float  QSincLstFPSampSontek;				//flow since last FPS measurement
												//calculated from Sontek data
    
    float  QCmlRating;							//cumulative flow from Rating Curve
    float  QSincLstFPSampRating;				//cumulative flow since last FPS 
												//measurement, calculated by Rating Curve
    
    unsigned long int  timeOfLstFPSamp;         //unix time of last composite sample 
    unsigned long int  timeOfLstFlowMeas;       //unix time of last flow measurement 
  } Flow;
  
///Parameters to be loaded from the SD card configuration file
///so far these are not actively used in the program
///need to add function call and mechanism to utilize data (either as the structure
///or assign data from structure to global variables
  struct parameters {
  String SiteID;			//short site identification name
  ///Main Sample System Timing Variables
  int PumpTime;//Time to run pump to fill lines with fresh sample water (seconds)
  int PumpToProbe;//Time to run pump to fill manta vessel and scan cuvette (seconds)
  int PurgeTime;//Time to run pump (reverse) to purge lines and vessel (seconds)
  int scanCleaningTime;//Delay between Scan cleaning signal and reading(seconds)
  int mantaDelayTime; //Time to accommodate wander of Manta and Scan Clocks (seconds)
  int measurementTime;//Another factor to accommodate timing (unused?) (seconds)
  int FPSTest;        //A flag to force testing of FPS system
  int FPSTestSamples; //number of samples to collect when FPSTest flag is set
  //int delayTime;//Hm. Same deal-- no longer exists in code
 ///Flow Proportional Sample System Timing Variables
  //int FPSPumpTime;//Time to pump to fill lines up to FPS Tee (seconds)
  //int FPSPurgeTime;//Time to purge same lines (seconds)
  int FPSCollectTime;//Wait time to drain FPS metering vessel into composite (seconds)
  int FPSDrainTime;//Wait time to drain excess from metering vessel(seconds)
  int FPSEquilibrateTime;
  int FreshWaterRinseTime;
  int FPSOperation;
  int AcidPumpTime;//Duration to run small pump for cleaing cuvette (seconds)
  int sontekInterval;//Interval at which to collect Sontek data 
 					 //(and decide whether or not to FPS) (MINUTES)
					 //not really used right now
  int SI;			//interval at which to pump to Manta system in absence of scan signal
  
  ///Flow Proportional System Collection Thresholds
  int TimeThreshold;//Time threshold for Flow proportional sample. If exceeded collect-regardless of cumulative flow
  int QThreshold;//Flow threshold (m3) when exceeded collect Flow Proportional Sample
  int TempThreshold;
  int FakeFlow;
} param;
 
 int SI                =15;            //consider having it be calculated in the program?
 int sontekInterval    =1; //interval at which to run flow pro
 int TimeThreshold     =1440;   //Time threshold where we want to take a FPS sample,
 int TempThreshold     =0;							//even if flow has been LOW
 int FPSOperation      =1;                                                     //in minutes
 int QThreshold        =1000; 	//Flow threshold for taking an FP Sample--either in a 
							//single timestep or over multiple timesteps
                                                        //m^3
                                                        
 unsigned int PumpTime          = 45;                        //pump in seconds to prime lines
 unsigned int PumpToProbe       = 5;                      //extra time to fully rinse probe
 unsigned int PurgeTime         = 45;                 //purge sample from lines (return to source)
 unsigned int scanCleaningTime  = 65;                //seconds to wait between receipt of clean signal
                                                         //and execution of scan spectroscopy
 unsigned int mantaDelayTime         = 60;//60;                  //Time to add to allow for wander in Manta RTC
 unsigned int measurementTime  = 10;                 //not used
 unsigned int FPSpurgeTime           = 10;                      //time to add to purge FlowProportionalSampler
//const unsigned int delayTime=10;                         //not used
 unsigned int FreshWaterRinseTime    = 4;                //TIME TO RUN PUMP TO CLEAN LENSES
 int FPSTest=0;                                          //set flag on SD card to test FPS when it is 1, samples will be collected
 int FPSTestSamples=2;                                    //tell it how many samples to take when FPStest is set (can't be more than 5, I don't think)

int FPSCollectTime            =5; 		//duration to pump for a sample for the FPS 
int FPSDrainTime              =15;  		//duration to wait for 
int FPSEquilibrateTime        =10; 	//duration to wait for excess to run out of FPS 							//sample metering unit
 unsigned int AcidPumpTime=2; //time to pump to fill cuvette with acid     
int FakeFlow=0;

//Parameters not read from config file
//File Names
 char systemLogFile[12] = "system.log"; //NAME FOR THE LOG FILE WRITTEN FOR THE 
										//Arduino system
 char sontekLogFile[12] = "sontek.log"; //NAME FOR THE LOGFILE CONTAINING SONTEK 
										//MEASUREMENT DATA
 char siteID[10] ="DEFAULT";           //NAME FOR THE SITE TO BE INCLUDED IN SYSTEM 
										//LOG
     
                                   
/****************     PREPARE GAUGE CAM SYSTEM  **********************************/
//SoftwareSerial cameraconnection = SoftwareSerial(2, 3);
//Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);
//const uint8_t imageSize=VC0706_640x480;

/****************     PREPARE SD CARD *******************************************/
#define chipSelect 53          //enable writing to SD card. SS is 10 for adafruit card

/****************     PREPARE REAL TIME CLOCK ***********************************/
Chronodot RTC;
DateTime now;

/****************     PREPARE I/O SYSTEM      ***********************************/
///INPUT VARIABLES
const int probeSignal   = A14;     //read Scan signal(digital read) high=sample
const int thermistor    = A13;     //read temp from IC (analog read)
const int batteryVoltage= A15;     //read Vbat via voltage divider(analog read)
const int floatSwitch   = A12;     //detect full FPS tank(digital read) 
	                           //low=full high=not full
const int TestCycle = A0;       //an input variable for runniung a test cycle

///OUTPUT VARIABLES
const int systemPump  = 27;        //pump to instrument     large peristaltic pump
const int systemPurge = 28;        //pump in reverse 	  	large peristaltic pump
const int acidPump    = 34;        //pump forward  			small peristaltic pump
const int acidPurge   = 36;        //pump reverse           small peristaltic pump
								   //valves are all 12vDC gravity valves
const int WaterQualityValve     = 23;  //connect main pump to sample system            
const int FPSCollectValve 	= 24;  //connects main pump to FPS metering vessel    
const int FPSCollectDrainValve  = 25;//drains FPS into collection tank             
const int ScanDrain             = 22;        //valve under ScanProbe
const int CleanWaterRinseValve  = 26;// valve that allows windshield washer pump to run
const int CleanWaterRinsePump   = 29;  //windshield washer pump that cleans scan lenses


//DECISION VARIABLES READ FROM SONTEK AND USED TO KEEP TRACK OF FLOW AND FPS SAMPLING
//This structure is a little bit redundant (it overlaps heavily with "decisionVariables" 
//array but this enhances readability, so I am inclined to keep it.
struct waterQual
{
   int sampleNumber;                    //from Sontek sample number since power cycle
   float flow;                          //from Sontek calculated for flume Area*Velocity 
   float stage;                         //depth [m] of water at site of Sontek 
   float meanVelocity;                  //
   float indexVelocity;                 //
   float waterDepth;                    //
   float waterTemp;                     //
   float timeSinceLastSample;           //
   float Qsontek;                       //same as flow
   float QsinceLastSampleSontek;        //
   float CumulativeQSontek;             // 
   float Qratings;                      //
   float QsinceLastSampleRatings;       //
   float CumulativeQRatings;            //
    
  } stream;
  
  
  
 ///GLOBAL VARIABLES
  int sampleCyclesSinceLastPowerCycle=0;//keep track of when we lose power...just for fun

  
  int errorCodes[7]={0,0,0,0,0,0,0};    //0 SD card 	0 is OK, 1 if error
                                        //1 Battery
                                        //2 Thermistor
                                        //3 RTC temp
                                        //4 Float switch
                                        //5 Sontek communication success
										//6 Percent Sontek submerged
                                        
                                        
  int   decisionVarINDX[7]={2,100,102,104,110,112,114}; //Index values for Sontek Registers 
													//that contain data of interest 
													//2	serial #
													//100 flow from(user defined channel)
													//102 Stage
													//104 mean velocity
													//110 index velocity
													//114 water temperature 
													
  float sensorValues[4];	//0 vBat (practically 5 to 14.4 volts)
							//1 Thermistor (Deg C)
							//2 RTC temp (Deg C)
							//3 float switch (0 or 1)
					  
 float decisionVariables[16];  	//It might be worthwhile defining these data as a 
								//structure too...they are  opaque listed in an array
      /*  FROM SONTEK
             decision variables: 
               0   2 sample                       [#]
               1   100 Flow                       [M3 S-1]
               2   102 Stage                      [M]
               3   104 Mean Velocity              [M S-1]
               4   106 Total Volume               [M3]
               5   112 Cross sectional Area       [m2]
               6   114 water temp                 [degC]
                   
               7   time since last sample         [sec]
                                      
               8   flow sontek                     [same as 1]
               
               9   this timestep Q SONTEK        calculated
               10   flow since last sample sontek calculated
               11  cumulative flow SONTEK        calculated
                    
               12  this timestep Q RATING CURVE  calculated
               13  flow rating curve             calculated
               14  flow since last sample rating calculated
               15  cumulative flow RATING CURVE  calculated

                    */

/**************************************************************************************/
/*************************** SETUP ROUTINE 		 **************************************/
/**************************************************************************************/
/**************************************************************************************/

  void setup(){
    //START SERVICES
    Serial.begin(9600);        //start serial monitor interface
    sontek.begin(19200);       //start upt the Modbus device
    Wire.begin();              //turn on i2c bus
    RTC.begin();               //turn the clock interface on
      
    //DECLARE PINMODE OUTPUTS
    pinMode(chipSelect, OUTPUT);       //turn on SD card SS 
    pinMode(systemPump,OUTPUT);
    pinMode(systemPurge,OUTPUT);
    pinMode(acidPump,OUTPUT);
    pinMode(acidPurge,OUTPUT);
    pinMode(WaterQualityValve,OUTPUT);
    pinMode(FPSCollectValve,OUTPUT);
    pinMode(FPSCollectDrainValve,OUTPUT);
    pinMode(ScanDrain,OUTPUT);
    pinMode(CleanWaterRinsePump,OUTPUT);
    pinMode(CleanWaterRinseValve,OUTPUT);
    // pinMode(FPSpurgeValve,OUTPUT);
    // pinMode(FPSpumpValve,OUTPUT);

      
	//DECLARE PINMODE INPUTS  
    pinMode(thermistor,INPUT);
    pinMode(floatSwitch,INPUT);
    pinMode(batteryVoltage,INPUT); 
    pinMode(probeSignal,INPUT);
    pinMode(TestCycle,INPUT_PULLUP);
	//MAKE SURE OUTPUTS ARE LOW (THIS IS PROBABLY UNNECESSARY)
//change this section for 16 Relay board--low signal trips the relays
//write everything HIGH
    digitalWrite(floatSwitch,HIGH);
    digitalWrite(systemPump,HIGH);
    digitalWrite(systemPurge,HIGH);
    digitalWrite(WaterQualityValve,HIGH);
    digitalWrite(FPSCollectValve,HIGH);
    digitalWrite(FPSCollectDrainValve,HIGH);
    digitalWrite(ScanDrain,HIGH);
    digitalWrite(acidPump,HIGH);
    digitalWrite(acidPurge,HIGH);
    digitalWrite(CleanWaterRinseValve,HIGH);
    digitalWrite(CleanWaterRinsePump,HIGH);
	//  digitalWrite(FPSpump,LOW);
	//  digitalWrite(FPSpump,LOW);
      
    //INDENTIFY THE INTERNAL REFERENCE HERE WE USE 1.1 V (3.3 ALSO EXISTS)  
    analogReference(INTERNAL1V1);    //1.1v internal reference for voltage
	
	//ACTIVATE THE SD CARD
    digitalWrite(chipSelect,HIGH);   //Tell the SD Card it is needed
  
      if (!SD.begin(chipSelect)) {//IF THE SD DOES NOT START
          Serial.println("Card failed, or not present");
          errorCodes[0]=1;  //add error to code, 
							//(although, it won't help, as they won't get logged ......
          //return;
        } //END  if (!SD.begin(chipSelect))

 //READ STATUS FROM SENSORS
  readSensors(sensorValues); 	//collect thermistor temp, RTC temp, float valve status, 
								//battery level. Update error codes as they are read
              
  //READ VALUES FROM EEPROM (NONVOLATILE MEMORY)
  EEPROM_readAnything(10, Flow);       //read flow data from Arduino EEPROM
  if(isnan(Flow.QCmlSontek)){
   Flow.QCmlSontek=0;
  };
  if(Flow.timeOfLstFlowMeas==0){       //If we reset EEPROM update time to present 
        Flow.timeOfLstFlowMeas=get_unixtime();
        Flow.timeOfLstFPSamp=get_unixtime();
    }
  //READ SETTINGS FROM SD CARD, IF FILE EXITS
getSettings("config.txt");  
  
  
  
  
//GIVE BRIEF UPDATE ON CONDITIONS 
    Serial.println("EEPROM memory read");
    Serial.print("cumulative flow since Sontek deployment: ");
    Serial.println(Flow.QCmlSontek);
    Serial.print("minutes since last composite sample: ");
    Serial.println((get_unixtime()-Flow.timeOfLstFPSamp)/60);
    Serial.print("flow since last composite sample: ");
    Serial.println(Flow.QSincLstFPSampSontek);
}



/**************************************************************************************/
/**************************		 MAIN LOOP		 **************************************/
/**************************************************************************************/
/**************************************************************************************/
void loop(){
   int collect=0; //used to count and record the number of samples collected in 
				  //FPS sample routine
				  
   long int counter=millis(); //IS THIS ACUTALLY USED ANYWHERE?
   DateTime watch=RTC.now();
   int sample=0;
   
//IF USING RTC SET SAMPLE HIGH EVERY SAMPLE INTERVAL
//(SI IS IN MINUTES, SO CONVERT TO SECONDS BEFORE MAKING THE COMPARISON)

   //when run by the scan probe, lets not, trigger sampling by the RTC
                                               //THIS MIGHT INTERFERE IF SCAN WANDERS...                                                          
    if (get_unixtime()%(60)==0){           //EVERY ONCE IN A WHILE REPORT STATUS
    
                // if (get_unixtime()%(SI*60)==0){sample=1;}   //EVERY SAMPLE INTERVAL GET READY TO RUN
                 timeStamp(); 
                 readSensors(sensorValues);  
                 Serial.print("approximate time to next sample: ");
                 Serial.println(SI-get_unixtime()%(SI*60)/60);//TIME TO NEXT SAMPLE
               //  pollSontek();
                 delay(900); //wait so the report is only given once
               }//end get_unixtime()%(60)==0)
			   
///IF SAMPLE SIGNAL IS GIVEN BY PROBE OR BY RTC TENTATIVELY ENTER THE SAMPLE CYCLE
if (digitalRead(probeSignal)|| !digitalRead(TestCycle) || sample){
         delay(1000); //WAIT SOME MS TO DEBOUNCE THE SIGNAL
                     //some indication that we need to wait for 2 seconds to make sure
                     //here just 1 second
                     //that the cleaning signal is the one we want, rather than just
                     //the probe being turned on.
		 
   ///todo add get_percenSubmerged();
   ///if this is less than 100% don't sample
      if( (allClear()<2) && (digitalRead(probeSignal) || !digitalRead(TestCycle)|| sample )){
///IF SIGNAL IS RECEIVED LONG ENOUGH OR RTC CALLS A SAMPLE AND THE TEMP IS OK
///ENTER THE STANDARD SAMPLE ROUTINE
            timeStamp();                      //GIVE FEEDBACK FOR TIME KEEPING
            Serial.println("pumpRoutine");
            sampleCyclesSinceLastPowerCycle++;//KEEP TRACK OF TIME SINCE POWER CYCLE
            readSensors(sensorValues);        //READ SENSORS TO SEE IF IT IS OK TO SAMPLE
            
			//UNIXTIME IS EASIER OF HANDLING CALCULATIONS            
            unsigned long int unixStartTime=get_unixtime(); //START TIME FOR SAMPLE CYCLE
            
            timeStamp();                       //PRINT TIME FOR SERIAL FEEDBACK
//**added this bit to have the scan drain open during purge, then closed during pump to probe
            Serial.println("purge lines");
            digitalWrite(ScanDrain,LOW); //delay (200);
            pump(systemPump,PumpTime,WaterQualityValve);//PUMP TO PROBE
            Serial.print("elapsed time= "); Serial.print(get_unixtime()-unixStartTime);  Serial.println();
            digitalWrite(ScanDrain,HIGH);
            Serial.println("pump to probe");
            pump(systemPump,PumpToProbe,WaterQualityValve);
            Serial.print("elapsed time= "); Serial.print(get_unixtime()-unixStartTime);  Serial.println();
         
            
/// WAIT FOR SCAN TO MEASURE  
            Serial.println("wait for S::CAN");  
            while((get_unixtime()-unixStartTime)<(scanCleaningTime+measurementTime)){
                             //delay(500);
                 }//END while((get_unixtime()-unixStartTime)<mantaDelayTime...
            Serial.print("elapsed time= "); Serial.print(get_unixtime()-unixStartTime);  Serial.println();
         
 //DRAIN the SCAN VESSEL
             Serial.println("Drain Scan");
             digitalWrite(ScanDrain,LOW);
             delay(3000);
             digitalWrite(ScanDrain,HIGH);
             Serial.print("elapsed time= "); Serial.print(get_unixtime()-unixStartTime);  Serial.println();
         
//WAIT FOR MANTA TO MEASURE'
            
            Serial.println("wait for Manta");
             while((get_unixtime()-unixStartTime)<(scanCleaningTime+measurementTime+mantaDelayTime)){
                 //delay(500);
             }//end while((get_unixtime()-unixStartTime)<(scanCleaningTime+measurementTime+mantaDelayTime))
             Serial.print("elapsed time= "); Serial.print(get_unixtime()-unixStartTime);  Serial.println();
         
 
 ///Dial up the sontek, download and store data.                          
            pollSontek();
															
///CALCULATE NUMBER OF FP SAMPLES TO COLLECT   
           int  numberOfSamples=calculateFlowProportionalSample(); 
				 //USES SONTEK DATA TO CALCULATE WHETHER OR NOT TO SAMPLE
				 //RETURNS NUMBER OF SAMPLE VOLUMES TO DRAW
				 //UPDATES VARIABLES IN EEPROM
  if (FPSTest){ 
                numberOfSamples=FPSTestSamples;    
          }  
///IF NECESSARY, EXECUTE FLOW PROPORTIONAL SAMPLE ROUTINE
if (FPSOperation){//turn off FPS operation if we want to 
                        int Danger=digitalRead(floatSwitch);
                        Serial.println("float Switch");
                        Serial.println(Danger);
			if(numberOfSamples && Danger){
					collect = collectFlowProportionalSample(numberOfSamples);
				}//END if(numberOfSamples){
			else {
					collect=numberOfSamples; //should be zero 0
				}//END else
	}//end if(FPSOperation){

    
  delay(5000);
  Serial.print("elapsed time= "); Serial.print(get_unixtime()-unixStartTime);  Serial.println();
         
///ALL DONE COLLECTING DATA AND WATER:  PURGE SYSTEM
             digitalWrite(ScanDrain,LOW);				//VALVE UNDER SCAN
          //   digitalWrite(FPSCollectValve,LOW);        //VALVE TO FPS METERING
             digitalWrite(WaterQualityValve,LOW);		//VALVE TO MANTA AND SCAN
            // digitalWrite(FPSCollectDrainValve,LOW);   //VALVE TO FPS COLLECTION
				delay(200);
             pump(systemPurge,PurgeTime,WaterQualityValve); //RUN THE PUMP
				delay(200);
		//	 digitalWrite(FPSCollectValve,LOW);         //VALVE TO FPS METERING 
                //       digitalWrite(FPSCollectDrainValve,HIGH);    //VALVE TO FPS COLLECTION
			 digitalWrite(ScanDrain,HIGH);				//VALVE BENEATH MANTA
			 digitalWrite(WaterQualityValve,HIGH);		//VALVE TO SCAN AND MANTA
                 //        digitalWrite(FPSCollectValve,HIGH);
 ///clean the lenses with a burst of water
 Serial.print("elapsed time= "); Serial.print(get_unixtime()-unixStartTime);  Serial.println();
 Serial.println("wash lenses");        
   //elay(1000);
   digitalWrite(ScanDrain,LOW);
   digitalWrite(CleanWaterRinseValve,LOW);
        delay(200);
    digitalWrite(CleanWaterRinsePump,LOW);
            long int hello=millis();
            while ((millis()-hello)<FreshWaterRinseTime*1000){
                 
                }   
          digitalWrite(CleanWaterRinsePump,HIGH);
                    delay(100);
          digitalWrite(CleanWaterRinseValve,HIGH);
          delay(2000);
          digitalWrite(ScanDrain,HIGH);
        //  digitalWrite(FPSCollectValve,LOW);
 Serial.print("elapsed time= "); Serial.print(get_unixtime()-unixStartTime);  Serial.println();
                

///PUMP ACID TO CLEAN CUVETT--if you want this routine is not defined yet
            //pump(acidPump,AcidPumpTime,0); 
			//this is not plumbed yet, but the pump is hooked into the system
      
///WRITE SYSTEM LOG FILE        
           writeSystemLogFile(0,collect,numberOfSamples);  //
           
///open all valves and allow any extra water to drain through--especially in FPS sample loop
 //run the pump forward and revers to move water through a couple catch points
 DrainAllValves(2000,3);
 Serial.print("elapsed time= "); Serial.print(get_unixtime()-unixStartTime);  Serial.println();
         
		   
        }//END  if( (allClear()<2) && (digitalRead(probeSignal) || sample ))

///IF TEMPERATURE IS TOO COLD TO RUN THE PUMPS GET FLOW MEASUREMENTS 
        else if(allClear()>=2){
            pollSontek();        //CHECK THE FLOW
			//CALCULATE SAMPLES (ROUTINE WILL SAY NO SAMPLING);
            long int  numberOfSamples=calculateFlowProportionalSample();   
			collect=numberOfSamples;
            writeSystemLogFile(0,collect,numberOfSamples); 	//WRITE THE RECORD
			//[WATER DEPTH],[#SAMPLES COLLECTED],[NUMER OF SAMPLES CALLED FOR]
			  
          }
            
 ///RESET COLLECT AND SAMPLE VARIABLES                   
		 collect=0;              //reset Collect Variable
		 sample=0;               //RESET SAMPLE TRIGGER
	}//end  while ((digitalRead(probeSignal)==HIGH)||sample==1)

  for (int j=0;j<=6;j++){
       errorCodes[j]=0;     //RESET ERROR CODE REGISTERS
      } //END for (int j=0;j<=6;j++){

}//END void loop(){


/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/

/// A CENTRAL LOCATION TO READ SENSOR VALUES (TEMP, TEMP, VBAT, FLOATSWITCH) AND
///UPDATE ERROR CODES RELATED TO THIER FUNCTION IN THE SYSTEM
///WHILE THIS FUNCTION IS A BIT OF A WRAPPER, I AM KEEPING IT FOR NOW BECAUSE IT OFFERS 
///GOOD SERIAL FEEDBACK FOR DEBUGGING, AND CENTRALIZED ERROR CODE EVALUATION
float readSensors(float* sensorValues){
   char conversion[10]; //variable to hold string conversion of data values
  Serial.println("reading sensors");
  //READ BATTERY VOLTAGE FRON VIN
          sensorValues[0] = get_vBatt();
		  //Serial.println(get_vBatt());
          if (get_vBatt()<12){
				errorCodes[1]=1;
				}//END  if (get_vBatt()<12)
				
  //READ THERMISTOR VOLTAGE CONVERT TO TEMP
          sensorValues[1] = get_tempThermistor();
		  //Serial.println(get_tempThermistor());
          if (get_tempThermistor() < -5){
				errorCodes[2]=1;
				}//END if (get_tempThermistor() < 0)
				
   //READ TEMPERATURE FROM RTC
            sensorValues[2] = get_tempRTC();
			//Serial.println(get_tempRTC());
           if (get_tempRTC()<-5){
				errorCodes[3]=1;
				}//END  if (get_tempRTC()<0)
				
   //READ STATUS OF FPS FLOAT SWITCH
           sensorValues[3] = get_floatSwitch();
		   //Serial.println(get_floatSwitch());
           if (get_floatSwitch()==0){ //0 WHEN BOTTLE IS FULL 1 WHEN NOT
				errorCodes[4]=1;
				}//END  if (get_floatSwitch()==0)

				Serial.println(get_unixtime());
				
   //CONVERT FLOAT TO STRING  
          dtostrf(sensorValues[0],3,4,conversion);   
               Serial.print("Battery Voltage: ");
               Serial.print((analogRead(batteryVoltage))); 
               Serial.print(" "); 
               Serial.print(conversion);
               Serial.print("\n");
          
          dtostrf(sensorValues[1],2,4,conversion);
               Serial.print("Thermistor: "); 
               Serial.print(analogRead(thermistor)); 
               Serial.print(" "); 
               Serial.print(conversion); 
               Serial.print("\n");
          
          dtostrf(sensorValues[2],3,4,conversion);
               Serial.print("Chronodot Temp: ");
               Serial.print(conversion); 
               //Serial.print(" "); 
               //Serial.print(conversion); 
               Serial.print("\n");
          
          dtostrf(sensorValues[3],3,0,conversion);
               Serial.print("FloatSwitch: ");
               Serial.print(digitalRead(floatSwitch)); 
              // Serial.print(" "); 
              // Serial.print(conversion); 
               Serial.print("\n");
               
        
      //         lowTemp=errorCodes[2]+errorCodes[3];//assign any thermal errors here
	  
}//END float readSensors(float* sensorValues){

/**************************************************************************************/
/***************** FUNCTION TO READ UNIXTIME FROM RTC *********************************/
/**************************************************************************************/

unsigned long int get_unixtime(){
    DateTime  now = RTC.now();  //DECLARE A RTC OBJECT
	unsigned long int time = now.unixtime();
	return time;			//RETURN  TIME IN SECONDS SINCE 1/1/1970 00:00:00
}//END unsigned long int get_unixTime()

/**************************************************************************************/
/************* FUNCTION TO READ TEMPERATURE IN C FROM RTC *****************************/
/**************************************************************************************/

float get_tempRTC(){
	DateTime now = RTC.now();//DECLARE A RTC OBJECT
	float tempC = now.tempC();
	return tempC;			//RETURN THE TEMPERATURE
	
}//END float get_tempRTC()

/**************************************************************************************/
/********** FUNCTION TO READ VALUE OF THERMISTOR **************************************/
/**************************************************************************************/

float get_tempThermistor(){
	int tempA = analogRead(thermistor); 	//READ THERMISTOR VALUE (0 TO 2048
	float tempB = ((1.1*100*tempA)/1023-32)*5/9;
    return  tempB;	//RETURN CONVERTED TO DEGREES C
 }//END float get_tempThermistor()

 /**************************************************************************************/
/*********** FUNCTION TO READ BATTERY VOLATE FROM VOLTAGE DIVIDER *********************/
/**************************************************************************************/
 
float get_vBatt(){
 float vBatt = (analogRead(batteryVoltage));	//READ BATTERY VALUE
 float vBattF = (vBatt/1023*14.4*1.1);
 return vBattF;			//RETURN CONVERTED TO VOLTS
}//END float get_vBatt()

/**************************************************************************************/
/**************** FUNCTION TO READ STATUS OF FLOAT SWITCH *****************************/
/**************************************************************************************/

int get_floatSwitch(){
int floatSwitch=digitalRead(floatSwitch);	//READ FLOAT SWITCH STATE
return floatSwitch;    						//RETURN VALUE (1 if empty OR 0 if full)
}//END int get_floatSwitch()

/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
///WRITE DATA FROM ARDUINO SYSTEM TO THE LOG FILE THIS INCLUDES VARIABLES USED TO MAKE 
///FPS COLLECTION DECISIONS, SYSTEM STATUS, TIME, RESTARTING, BATTERY LEVEL, TEMP
///CUMULATIVE FLOW ETC. THUS IT WILL BE GOOD FOR TROUBLESHOOTING

void writeSystemLogFile(float waterDepth, float collect, int numberOfSamples){
//PREPARED DATA FOR FILE HEADER ABBREVIATIONS OF VARIABLES AND UNITS ARE HERE 
//EVERYTHING IS TAB DELIMITED, SO EXCEL SHOULD READ IT IN WELL
//MUST BE CHANGED IF THE OUTPUTS ARE CHANGED
    char* registerNames[]={"YYYY","MM","DD","HH","mm",
                          "SS","siteID","ArdID","Ver","VBat",
                          "therm","DegC", "FPS","cycle","err",
                          "samp#","flow","stage","Mvel","Volume",
                          "xSect","H2OT","TSLO","TSLFPS","VolII",
                          "QSLS","CQS","QR","QSLSR","CQR", 
                          "#SS","#SR","allClear"};
    char* units[]={"[YYYY]","[MM]","[DD]","[HH]","[mm]",
                   "[SS]","[ID]","[ARD]","[Ver]","[v]",
                   "[degC]","[degC]","[#]","[#]","[1-7]",
                   "[#]","[m3 s-1]","[m]","[m s-1]","[m3]",
                   "[m2]","[degC]","[s]","[s]","[m3]",
                   "[m3]","[m3]","[m3]","[m3]","[m3]",
                   "[#]","[#]","[1,2,3]"};
                          
          
	if (!SD.exists(systemLogFile)){ 					//IF THE LOG FILE DOES NOT EXIST
		File dataFile=SD.open(systemLogFile,FILE_WRITE);//CREATE THE LOG FILE

                for (int j=0;j<32;j++){							//FOR EACH LOG VARIABLE
                      dataFile.print(registerNames[j]);			//PRINT A SHORT VARIABLE NAME
                      dataFile.print("\t");      				//ADD A TAB
                      }//END for (int j=0;j<30;j++)
                      
        		dataFile.print("\n");							//ADD NEWLINE
        		
                for (int j=0;j<32;j++){							//FOR EACH LOG VARIABLE
                      dataFile.print(units[j]);					//PRINT UNITS
                      dataFile.print("\t");      				//ADD A TAB
                      } //END  for (int j=0;j<30;j++)
                      
               dataFile.print("\n");							//ADD A NEWLINE
               dataFile.close();   								//CLOSE THE FILE
       }//END 	if (!SD.exists(systemLogFile))
  
  
      char conversion[10];			        //MAKE A CONTAINER FOR CONVERSION
	
      DateTime now = RTC.now();				//PROVIDE CURRENT TIME FOR LOG FILE
  
      File dataFile=SD.open(systemLogFile, FILE_WRITE);   //open log file	  
        
        if (dataFile){			 //IF THE DATA FILE OPENED WRITE DATA INTO IT
			char dataString[27]; //A CONTAINER FOR THE FORMATTED DATE
			
			//PRINT THE DATE (YYYY MM DD HH mm SS) TO dateString variable
			int a = sprintf(dataString,"%d\t%02d\t%02d\t%02d\t%02d\t%02d\t",now.year(), 
					now.month(), now.day(),now.hour(),now.minute(),now.second());
			
				
			dataFile.print(dataString);	//WRITE THE DATE STRING TO THE LOG FILE	
			
			dataFile.print(siteID);  	//write siteID TO THE LOG FILE
			dataFile.print("\t");	 	//ADD A TAB
			
			char id[5];					//MAKE A VARIABLE FOR THE ARDUINO ID
			EEPROM_readAnything(0, id);	//READ ARDUINO ID FROM EEPROM
			dataFile.print(id);  		//write ArduinoID TO LOG FILE
			dataFile.print("\t");		//ADD A TAB
			
			dtostrf(VersionNumber,2,2,conversion); 	//CONVERT PROGRAM VERSION TO STRING
			dataFile.print(conversion);				//WRITE TO LOG FILE
			dataFile.print("\t");					//ADD A TAB
			
			dtostrf(sensorValues[0],3,4,conversion);//CONVERT VOLTAGE TO STRING
			dataFile.print(conversion);				//WRITE STRING TO LOG FILE
			dataFile.print("\t");					//ADD A TAB
		   
			dtostrf((sensorValues[1]),2,4,conversion);//CONVERT THERM DEG C TO STRING
			dataFile.print(conversion);				//WRITE STRING TO LOG FILE
			dataFile.print("\t");					//ADD A TAB
		   
			dtostrf(sensorValues[2],3,4,conversion);//CONVERT RTC DEG C TO STRING
			dataFile.print(conversion);				//WRITE STRING TO LOG FILE
			dataFile.print("\t");					//ADD A TAB
           
			dataFile.print(numberOfSamples);         //WRITE NUMBER OF FPS SAMPLES 
													//COLLECTED IN THIS INTERVAL
			dataFile.print("\t");					//ADD A TAB
           
													//WRITE CYCLES SINCE LAST PC
           dataFile.print(sampleCyclesSinceLastPowerCycle);  
           dataFile.print("\t"); 					//ADD A TAB
          
           for (int i=0;i<7;i++){					//FOR EACH OF THE ERROR CODES
               dataFile.print(errorCodes[i]);		//WRITE TO LOG FILE
               dataFile.print(" ");					//ADD A SPACE
               }//END for (int i=0;i<7;i++)
			   
            dataFile.print("\t");					//THEN ADD A TAB
            
 //HERE ADD THE CODE FOR STRUCTURE DECISION VARIABLES
 //RATHER THAN BEING LAZY AND PRINTING FROM THE VECTOR DECISION VARIABLES
 //WE COULD PRINT THEM ONE BY ONE WITH 16 INDIVIDUAL STATEMENTS
 
           for (int k=0; k<=16;k++){
				dataFile.print(decisionVariables[k],4);
				dataFile.print("\t");
			   }//END for (int k=0; k<=15;k++)
           
           dataFile.print(allClear()); 	//0 no problems,
										//1 full bottle
										//2 too cold,
									    //3 too cold AND bottle full  
										//4 Sontek not submerged
           dataFile.print("\t"); 
               
          dataFile.print("\n");  		//add final carriage return
          dataFile.close();				//close file
          Serial.println("successful File Writing");
         // Serial.println(dataString);
		}//END if (dataFile)
    
  
}//END void writeSystemLogFile(float waterDepth, float collect, int numberOfSamples)


/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
///A CENTRAL FUNCTION TO CONTROL THE SYSTEM PUMP IT OPENS AND CLOSES VALVES 
///I LIKE TO DO IT ALL IN A FUNCTION TO PREVENT OPERATOR MISTAKES, LIKE FAILING TO CLOSE 
///A VALVE OR THE LIKE
///Specify 0 for the valve to NOT activate a valve during pumping....
void pump(int activePump,long int duration, int valve) {
     if (valve>0){
        digitalWrite(valve,LOW);        //open valve to pump water to collection point.
         delay(200);                     //give it a change to open
         long int cycleTime=millis();    //initialize timer variable
         digitalWrite(activePump,LOW);  //turn pump on
                                         //wait
              while((millis()-cycleTime)/1000<duration){ 
              
              }
              
         digitalWrite(activePump,HIGH);  //turn pump off
         delay(300);                    //wait for pressure in lines to balance         
         digitalWrite(valve,HIGH);       //close the valve
  }
  else{//in case someone doesn't want to activate a vale, 
         long int cycleTime=millis();   //initialize timer variable
         digitalWrite(activePump,LOW); //turn pump on
         
              while((millis()-cycleTime)/1000<duration){
              
               }
         digitalWrite(activePump,HIGH);  //turn pump off
  }
  
}

/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
void pollSontek(){
     static uint32_t i;
     uint8_t j, result;
     uint16_t data[6];
	 
     char* registerNames[]={"Serial","Sample","YYYY","MM","DD","HH","mm","SS","flow",
				"stge","MVel","TVol","DepP","INDXV","XSecA","H20T","SStat","CXV","CZV",
				"LXV","RXV","Bat","Ptch","Roll","%sub","DtoS","Dpth","TotV+","TotV-",
				"ECell","SNRB1","SNRB2","SNRB3","SNRB4"};
				
     char* units[]={"[#]","[#]","[]","[]","[]","[]","[]","[]","[m s-1]","[m]", "[m s-1]",
				"[m3]","[m]","[m s-1]","[m2]","[deg C]","[ ]","[m s-1]","[m s-1]",
				"[m s-1]","[m s-1]","[v]","[deg]","[deg]","[%]","[m]","[m]","[m3]",
				"[m3]","[m]", "[dB]","[dB]","[dB]","[dB]"};
      
     i++;
     
    // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
    sontek.setTransmitBuffer(0, lowWord(i));
  
    // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
    sontek.setTransmitBuffer(1, highWord(i));
  
	Serial.println("poll Sontek Routine");
  
	//TODO find out why sontekLogFile doesn't work for the exists function
	if (!SD.exists(sontekLogFile)){			//IF THE LOG FILE DOES NOT EXIST MAKE ONE
		File dataFile=SD.open(sontekLogFile,FILE_WRITE); //CREATE FILENAME
		
		//WRITE A HEADER SO THAT SOMEONE LOOKING AT THE FILE WILL BE ABLE TO UNDERSTAND
		//WHAT IS IN IT--WILL OPEN IN EXCEL NICELY TAB DELIMITED WITH 2 HEADER LINES
		
        for (j=0;j<34;j++){									//FOR EACH LOG VARIABLE
              dataFile.print(registerNames[j]);				//WRITE THE SHORT NAME
              dataFile.print("\t");      					//ADD A TAB
			}//end for (j=0;j<34;j++)
		dataFile.print("\n");								//ADD A NEWLINE
        
		for (j=0;j<34;j++){									//FOR EACH LOG VARIABLE
              dataFile.print(units[j]);						//WRITE UNITS
              dataFile.print("\t");      					//ADD A TAB CHARACTER
              }//end for (j=0;j<34;j++)		
       dataFile.print("\n");								//ADD A NEWLINE
       
       dataFile.close();   									//CLOSE THE FILE 
       } //END 	if (!SD.exists(sontekLogFile))
  
  
   File dataFile=SD.open(sontekLogFile, FILE_WRITE);  		//OPEN EXISTING LOG FILE
          if (dataFile){ //IF FILE OPEN IS SUCCESSFUL READ AND STORE EACH DATA REGISTER
                  /**********THIS PART IS JUST FOR SERIAL OUTPUT***********/
				  for (j=0;j<34;j++){ 					//FOR EACH VARIABLE
                      Serial.print(registerNames[j]); 	//PRINT THE NAME
                      Serial.print("\t");   		  	//ADD A TAB
                      }//END for (j=0;j<34;j++)
                      Serial.println();   				//NEW LINE
					  
					  
 ///SONTEK REGISTERS 0-14 CONTAIN INTEGER DATA         
                for (int i=0; i<=14; i+=2){  
                    result=sontek.readInputRegisters(i,2); //read input registers
					//IF READ WAS A SUCCESS,
					//CONVERT DATA TO INTEGER VARIABLE AND WRITE TO SDCARD
                    if (result == sontek.ku8MBSuccess){ //SHUFFLE BYTE ORDER
							registerValue.b[0]=lowByte(sontek.getResponseBuffer(0));
                            registerValue.b[1]=highByte(sontek.getResponseBuffer(0));
                            registerValue.b[2]=lowByte(sontek.getResponseBuffer(1));
                            registerValue.b[3]=highByte(sontek.getResponseBuffer(1));
                         
                            if (i==0){
									long int HaileSelassie=registerValue.fval;
									Serial.print(HaileSelassie); Serial.print("\t");     
									dataFile.print(HaileSelassie);//dataString);
									dataFile.print("\t");
								} //END if (i==0)
							else { //convert float to int for display
									int RasTafari=registerValue.fval;
									char output[4];
									sprintf(output,"%02d",RasTafari);
									Serial.print(output); 
									Serial.print("\t");     
									dataFile.print(output);//dataString);
									dataFile.print("\t");
								}  //END ELSE
                        }//END if (result == sontek.ku8MBSuccess)
                    }//END for (int i=0; i<=14; i+=2)
                                        
///REGISTERS 100-150 CONTAIN MOSTLY FLOAT DATA (EXCEP 132--WHICH IS AN INTEGER)										
				for (int i=100; i<=150; i+=2){
					result=sontek.readInputRegisters(i,2);  //read input registers
                    //IF READ WAS A SUCCESS,
					//CONVERT DATA TO FLOAT VARIABLE AND WRITE TO SDCARD
					if (result == sontek.ku8MBSuccess){ //SHUFFLE BYTE ORDER
							registerValue.b[0]=lowByte(sontek.getResponseBuffer(0));
							registerValue.b[1]=highByte(sontek.getResponseBuffer(0));
							registerValue.b[2]=lowByte(sontek.getResponseBuffer(1));
							registerValue.b[3]=highByte(sontek.getResponseBuffer(1));

				//AN EXCEPTION SYSTEM STATUS BEST REPRESENTED AS AN INTEGER						
							if (i==116){//integer value stored here
									//UNION CONVERTS FLOAT TO INT
									int RasTafari=registerValue.fval;
									Serial.print(RasTafari); 
									Serial.print("\t");
									dataFile.print(RasTafari);//dataString);
									dataFile.print("\t");
								}//END if (i==116)
				//OTHERWISE STORE IT AS A FLOAT
							 else { //float values stored here
									Serial.print(registerValue.fval,6); 
									Serial.print("\t");
									dataFile.print(registerValue.fval,6);//dataString);
									dataFile.print("\t");
								} //END ELSE
				//REGISTER 132 HOLDS %INSTRUMENT SUBMERGED
							   if (i==132){//%of instrument submerged
									if (registerValue.fval<100){
									//IF THE INSTRUMENT IS NOT SUBMERGED NOTE IT AS AN 
									//ERROR: READINGS ARE HIGHLY SUSPICIOUS
											errorCodes[6]=1;
										}//END if (registerValue.fval<100)
								}//END  if (i==132)
							   
						}//END if (result == sontek.ku8MBSuccess
							  
					} //END for (int i=100; i<=150; i+=2)
					
                           Serial.println();
                           dataFile.print("\n");
                           dataFile.close();
///IF SONTEK READ IS A FAILURE						   
					if (result!=sontek.ku8MBSuccess){
						errorCodes[5]=1;
						}//END if (result!=sontek.ku8MBSuccess)
				}//END  if (dataFile)
  
  //GET THE DATA THAT WE USE TO MAKE DECISIONS REGARDING FLOW PARAMETERS
  //THESE ARE SPECIFIED IN THE DECISION VARIABLE INDEX ABOVE
  //FOR SOME REASON IT SEEMED EASIER TO JUST DO IT AGAIN, RATHER THAN SEPARATE THEM OUT
  //IN THE PREVIOUS FUNCTION
  
 /* //THIS IS THE DECLARATION COPIED FROM ABOVE
    int   decisionVarINDX[6]={2,100,102,104,110,112,114}; //Index values for Sontek Registers 
													//that contain data of interest 
													//2	serial #
													//100 flow from(user defined channel)
													//102 Stage
													//104 mean velocity
													//110 index velocity
                                                                                                        //112 totalvolume
													//114 water temperature
*/													
													
            for (int k=0; k<(sizeof(decisionVarINDX)/sizeof(decisionVarINDX[0]));k++){
                    result=sontek.readInputRegisters(decisionVarINDX[k],2);
	//CONVERT DATA TO FLOATING POINT VARIABLE AND ASSIGN TO DATA VECTOR
					//THIS IS ACCOMPLISHED VIA THE UNION DECLARED ABOVE
                    if (result == sontek.ku8MBSuccess){
							registerValue.b[0]=lowByte(sontek.getResponseBuffer(0));
							registerValue.b[1]=highByte(sontek.getResponseBuffer(0));
							registerValue.b[2]=lowByte(sontek.getResponseBuffer(1));
							registerValue.b[3]=highByte(sontek.getResponseBuffer(1));
					   }//END  if (result == sontek.ku8MBSuccess)
                      decisionVariables[k]=registerValue.fval;//USE THE FLOAT VALUE
                      //Serial.println(registerValue.fval,4);                 
            }//END for (int k=0; k<(sizeof(decisionVarINDX)/sizeof(decisionVarINDX[0]))
               

}//END void pollSontek(){


/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
//USE DECISION VARIABLES TO EVALUATE WHETHER OR NOT A FLOW PROPORTIONAL SAMPLE SHOULD
//BE COLLECTED, AND HOW MANY ALSO CONSIDERS TEMPERATURE, WATER DEPTH, AND BOTTLE FULLNESS
long int calculateFlowProportionalSample(){
     int samplesSontek;
     int samplesRating;
     char conversion[10];
	 
    long int Upresent=get_unixtime();//GET CURRENT TIME IN SECONDS
	
    //SECONDS SINCE LAST FLOW OBSERVATION WAS RECORDED
	long int timeSincLstFlowMeas  = (Upresent-Flow.timeOfLstFlowMeas);    
	//SECONDS SINCE LAST FPS WAS COLLECTED--USED FOR TimeThreshold
	long int timeSincLstFPSamp    = (Upresent-Flow.timeOfLstFPSamp);  
	
///UPDATE CUMULATIVE FLOW VARIABLES
    //Sontek Flow (M3S-1]*ELAPSED TIME = FLOW SINCE LAST MEASUREMENT
    float QSontek=decisionVariables[1]*timeSincLstFlowMeas; 

        Flow.QCmlSontek           += QSontek; 	//ADD NEW FLOW TO CUMULATIVE TOTAL
        Flow.QSincLstFPSampSontek += QSontek;	//ADD NEW FLOW TO FLOW SINCE LAST FPS
//If no SONTEK is attached, give the program fakeflow to test math and such
if(FakeFlow){
          Flow.QCmlSontek          +=  FakeFlow;
          Flow.QSincLstFPSampSontek +=  FakeFlow;
        }
///DEFINE A RATING EQUATION
//HERE 10-2-2013 I AM USING UNIFORM CHANEL WIDTH 0.2[M], APPROXIMATE FLOW 0.1 
//[M S-1] AND STAGE FROM SONTEK [M]
////old eqn// 2.03*pow(decisionVariables[2],2.089);	 
     float QRating=decisionVariables[3]*decisionVariables[5]*timeSincLstFlowMeas; 
       
        Flow.QCmlRating           += QRating; //ADD NEW FLOW TO CUMULATIVE TOTAL
        Flow.QSincLstFPSampRating += QRating; //ADD NEW FLOW TO FLOW SINCE LAST FPS

///SERIAL FEEDBACK FOR DEBUGGING REPORT THESE VALUES A FEW DIFFERENT WAYS
		Serial.print("time since last Flow Measurement: "); 
		Serial.print(timeSincLstFlowMeas/60); //report elapsed time in minutes 
		Serial.println();
		Serial.print("time since last Flow Prop Sample: "); 
		Serial.print(timeSincLstFPSamp/60); //report elapsed time in minutes 
		Serial.println();
    
///SERIAL OUTPUT OF SONTEK CALCULATIONS
		dtostrf(QSontek,6,4,conversion);					//CONVERT TO STRING
		Serial.print("QSontek: "); 							//GIVE SOME CONTEXT
		Serial.print(conversion); 							//REPORT VALUE
		Serial.println();									//ADD A NEWLINE

		dtostrf(Flow.QSincLstFPSampSontek,6,4,conversion); 	//CONVERT TO STRING
		Serial.print("QSontek Since last FPS: "); 		 	//GIVE SOME CONTEXT
		Serial.print(conversion);							//REPORT VALUE
		Serial.println();									//ADD A NEWLINE
		  
		dtostrf(Flow.QCmlSontek,6,4,conversion);			//CONVERT TO STRING
		Serial.print("cumulativeQSontek: "); 				//GIVE SOME CONTEXT
		Serial.print(conversion);							//REPORT VALUE
		Serial.println();  									//ADD A NEWLINE
          
///SERIAL OUTPUT OF RATING CALCULATIONS      
        dtostrf(QRating,6,4,conversion);					//CONVERT TO STRING
        Serial.print("Q rating: "); 						//GIVE SOME CONTEXT
		Serial.print(conversion);							//REPORT VALUE
		Serial.println();									//ADD A NEWLINE

        dtostrf(Flow.QSincLstFPSampRating,6,4,conversion);	//CONVERT TO STRING
        Serial.print("QRating Since last FPs: "); 			//GIVE SOME CONTEXT
		Serial.print(conversion);							//REPORT VALUE
		Serial.println();									//ADD A NEWLINE
          
        dtostrf(Flow.QCmlRating,6,4,conversion);			//CONVERT TO STRING
        Serial.print("cumulativeQRating: "); 				//GIVE SOME CONTEXT
		Serial.print(conversion);							//REPORT VALUE
		Serial.println();									//ADD A NEWLINE
          
///ADD THE VALUES USED TO MAKE THIS SAMPLE DECISION TO THE DECISION VARIABLE VECTOR 
         //HERE ADD CODE TO ADD STRUCTURE WATER QUALITY FOR EASIER READABILITY
		 //THE DECISIONVVARIABLE VECTOR IS A LITTLE BIT OPAQUE
          decisionVariables[7] = timeSincLstFlowMeas;            
          decisionVariables[8] = timeSincLstFPSamp;                               
          decisionVariables[9] = QSontek; 
          decisionVariables[10] = Flow.QSincLstFPSampSontek;
          decisionVariables[11] = Flow.QCmlSontek;
          decisionVariables[12] = QRating;
          decisionVariables[13] = Flow.QSincLstFPSampRating;
          decisionVariables[14] = Flow.QCmlRating;
          decisionVariables[15] = samplesSontek;
          decisionVariables[16] = samplesRating;
          
 ///REPORT THE THRESHOLDS THAT ARE BEING USED         
 Serial.print("Time threshold: "); Serial.println(TimeThreshold);
 Serial.print("FLow threshold: "); Serial.println(QThreshold);

///DECIDE WHETHER OR NOT TO SAMPLE, AND HOW MANY SAMPLES TO TAKE
	if(((timeSincLstFPSamp/60)>TimeThreshold || Flow.QSincLstFPSampSontek>QThreshold) && allClear()==0){
		//IF TIME IS GREATER THAN TIME THRESHOLD or FLOW IS GREATER THAN FLOW THRESHOLD
		//AND THE TEMPERATURE ISN'T TOO LOW AND THE BOTTLE ISN'T FULL, PROCEED WITH 
		//CALCULATING AND POSSIBLY TAKING A SAMPLE
           Serial.println(" take a sample");
//RECNT CHANGE IF BUGGY, CHECK		   
           samplesSontek = round(Flow.QSincLstFPSampSontek/QThreshold);
           samplesRating = round(Flow.QSincLstFPSampRating/QThreshold);
          decisionVariables[15] = samplesSontek;
          decisionVariables[16] = samplesRating;
		   if ((timeSincLstFPSamp/60)>TimeThreshold*1.1){
		   //INSTRUMENT HAS PROBABLY BEEN OFF, COLLECTION SHOULD HAVE BEEN TRIGGERED
		   //ALSO, COULD HAVE HAD TEMPERATURE ISSUE OR FULL BOTTLE
		   //EITHER WAY, CAN'T MAKE UP OLD FLOW ONLY USE Q FROM LAST TIME-STEP 
		   //TO CALCULATE SAMPLE
		   samplesSontek = round(QSontek/QThreshold);
		   samplesRating = round(QRating/QThreshold);
		   
		   }//end if ((timeSincLstFPSamp/60)>TimeThreshold*1.1)
           if (samplesSontek<1){  
					//IF THE TIME THRESHOLD (TimeThreshold) HAS TRIGGERED SAMPLING
                    //THERE IS NO REASON TO EXPECT THAT A FULL SAMPLE IS CALLED FOR, 
                    //LETSMAKE SURE THAT THE ROUND FUNCTION(ABOVE)DOESN'T ROUND DOWN THE 
                    //VALUEI DID CHOOSE ROUND OVER CEILING FUNCTION TO AVOID BIASING 
					//TOWARDS OVER-SAMPLING
                 samplesSontek=1;
				}//END  if (samplesSontek<1){ 
		   if (samplesRating<1){  
					//IF THE TIME THRESHOLD (TimeThreshold) HAS TRIGGERED SAMPLING
                    //THERE IS NO REASON TO EXPECT THAT A FULL SAMPLE IS CALLED FOR, 
                    //LETSMAKE SURE THAT THE ROUND FUNCTION(ABOVE)DOESN'T ROUND DOWN THE 
                    //VALUEI DID CHOOSE ROUND OVER CEILING FUNCTION TO AVOID BIASING 
					//TOWARDS OVER-SAMPLING
                 samplesRating=1;
				}//END  if (samplesRating<1){  
          //GIVE SERIAL FEEDBACK ON DECISION
          dtostrf(samplesSontek,6,4,conversion); 			//CONVERT TO STRING
          Serial.print("Number of samples from Sontek: ");  //PROVIDE CONTEXT
		  Serial.print(conversion); 						//SHOW VALUE
		  Serial.println();									//ADD NEWLINE
  
          dtostrf(samplesRating,6,4,conversion);			//CONVERT TO STRING
          Serial.print("Number of samples from Rating"); 	//PROVIDE CONTEXT
		  Serial.print(conversion); 						//PRINT VALUE
		  Serial.println();									//ADD NEWLINE
  
///IF A SAMPLE IS TAKEN RESET THE BOOKKEEPING PARAMETERS   
//hang on to the remainder in excess of the threshold
//BUT, if it was set by time threshold, set it to 0...
          if ((timeSincLstFPSamp/60)>=TimeThreshold){
                 Flow.QSincLstFPSampSontek=0;	//currentQ-Qthreshold					//RESET SONTEK FLOW COUNTER
                 Flow.QSincLstFPSampRating=0;   					//RESET RATING FLOW COUNTER
	      }
          timeSincLstFPSamp=0;
          Flow.timeOfLstFPSamp=Upresent;					//TIME OF LAST SAMPLE IS NOW
          Flow.QSincLstFPSampSontek=(round(Flow.QSincLstFPSampSontek)%QThreshold);	//currentQ-Qthreshold					//RESET SONTEK FLOW COUNTER
          Flow.QSincLstFPSampRating=(round(Flow.QSincLstFPSampRating)%QThreshold);   					//RESET RATING FLOW COUNTER
		}// END if(((timeSincLstFPSamp/60)>TimeThreshold || F...
		
///IF NO SAMPLE IS REQUIRED BY QTHRESHOLD OR TimeThreshold OR THE BOTTLE IS FULL
///THEN DON'T TAKE A SAMPLE		  
	else{
          Serial.println(" don't sample");
          samplesSontek=0; //RETURN 0 FOR NUMBER OF SAMPLES TO TAKE
          samplesRating=0; //RETURN 0 FOR NUMBER OF SAMPLES TO TAKE
			}// END ELSE

       
    Flow.timeOfLstFlowMeas=Upresent;//UPDATE THE TIME OF LAST FLOW MEASUREMENT
 
    EEPROM_writeAnything(10, Flow);  //WRITE FLOW VALUES

	Serial.println();
	Serial.println();
    Serial.println("Number of FPS sample volumes to collect: ");
    Serial.println(samplesSontek);
///RETURN THE NUMBER OF SAMPLE TO COLLECT
//return samplesRating;
return samplesSontek;  //the predicted time to spend sampling
                       
}//end calculateFlowProportionalSample() FUNCTION

/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
///PROVIDES SERIAL OUTPUT ABOUT TIME, TEMPERTURE AND THE LIKE
///GOOD FOR MAKING SURE RTC IS RUNNING AND CYCLES ARE EXECUTING ACCORDING TO PLAN
void timeStamp(){
  DateTime  now = RTC.now();         //start clock object "now"
  //Serial.println(timestamp);
 Serial.print(now.year(), DEC);
    Serial.print('/');
    if(now.month() < 10) Serial.print("0");
    Serial.print(now.month(), DEC);
    Serial.print('/');
    if(now.day() < 10) Serial.print("0");
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    if(now.hour() < 10) Serial.print("0");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    if(now.minute() < 10) Serial.print("0");
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    if(now.second() < 10) Serial.print("0");
    Serial.print(now.second(), DEC);
    Serial.print("\t");

    Serial.print(now.tempC(), 1);
    Serial.println(" degrees Celcius");
    //Serial.print(now.tempF(), DEC);
    //Serial.println(" degrees Farenheit");
    
    Serial.println();
}//END timeStamp() FUNCTION


/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
 ///READ THE IMPORTANT SYSTEM PROGRAMMING VARIABLES FROM A TEXT FILE LOCATED IN THE ROOT 
 ///DIRECTORY OF THE SD CARD.  THIS ENABLES THE USER TO NOT CONTIUNUALLY RELOAD THE FPS
 ///CORE PROGRAM ON THE ARDUINO. INSTEAD BY EDITING THE CONFIG FILE, AND RESTARTING THE
 ///SYSTEM PUMPING TIMES AND THRESHOLD VALUES CAN BE CHANGED AND TWEAKED
 void getSettings(char* settings){
  Serial.println("Loading Configuration");
 // Open the settings file for reading:
 File myFile;						//DECLARE A FILE
  myFile = SD.open(settings);		//OPEN THE FILE
  if(myFile){						//IF THE FILE OPENS PROCEED WITH PARSING DATA
    Serial.println("File Opened");	//GIVE SOME FEEDBACK
  char character;					//AN EMPTY CHARACTER
  String description = "";			//AND EMPTY STRING
  String value = "";				//ANOTHER EMPTY STRING
  boolean valid = true;				//BOOL FOR EVALUATING STATMENTS
  
  
          while (myFile.available()) { //READ FROM THE FILE UNTIL ITS EMPTY
            character = myFile.read(); //READ FIRST CHAR OF LINE
                if(character == '/') { //IF ITS A COMMENT READ THE WHOLE LINE
                    while(character != '\n'){
                            character = myFile.read();
                        }//END  while(character != '\n')
                   }//END if(character == '/')
				   
				//IF IT ISN'T A COMMENT, IT IS THE VARIABLE NAME
                else if(isalnum(character)) {//ADD EACH CHARACTER TO THE DESCRIPTION
                           description.concat(character);
                    } 
                else if(character =='=') {//IF IT IS AN EQUALS SIGN, 
										  //END THE VARIABLE NAME AND READ THE VALUE
                        Serial.print(description);
                        Serial.print(": ");
                          // START CHECKING THE VALUE FOR POSSIBLE RESULTS
                          // FIRST GOING TO TRIM OUT ALL TRAILING WHITE SPACES
                          do { character = myFile.read();
                                } while(character == ' ');
						  //EMPTY THE VALUE PARAMETER
                                  value = ""; 
                          //READ IN THE REST OF THE LINE
						  //WHILE THE LINE ISN'T OVER AND DATA REMAINS IN THE FILE
						  //CONTINUTE CONCATENATING CHARACTERS
								  while(character != '\n' && myFile.available()) { 
								            value.concat(character);               
                                            character = myFile.read();
                                      }//end while(character != '\n')
							//CREATE A BUFFER TO HOLD THE ASSEMBLED DATA
                                  char charBuf[value.length()+1];         
							//CONVER THE VALUE TO SOMETHING USEABLE  
                                  value.toCharArray(charBuf,value.length()+1);
                                  Serial.print(charBuf);
                                  Serial.println();
           
              //VALUE PAIR SHOULD BE CAPTURED AT THIS POINT
              //ASSIGN THEM TO REAL VARIABLES, OR TO A STRUCTURE
			  //BY MATCHING DESCRIPTION AND VARIABLE NAMES
                                  if (description == "siteID"){
                                    param.SiteID=value;
                                  }//END  if (description == "siteID")
                                  else if (description == "FPSOperation"){
                                    param.FPSOperation=atoi(charBuf);
                                  }//END  if (description == "siteID")
                                  else if (description == "PumpTime"){
                                    param.PumpTime=atoi(charBuf);
                                  } //END else if (description == "PumpTime")
                                   else if (description == "PumpToProbe"){
                                    param.PumpToProbe=atoi(charBuf);
                                  } //END else if (description == "PumpToProbe")
                                   else if (description == "PurgeTime"){
                                    param.PurgeTime=atoi(charBuf);
                                  } //END  else if (description == "PurgeTime")
                                  else if (description == "scanCleaningTime"){
                                    param.scanCleaningTime=atoi(charBuf);
                                  } //END else if (description == "scanCleaningTime")
                                  else if (description == "mantaDelayTime"){
                                    param.mantaDelayTime=atoi(charBuf);
                                  } //END else if (description == "mantaDelayTime")
                                   else if (description == "measurementTime"){
                                    param.measurementTime=atoi(charBuf);
                                  } //END  else if (description == "measurementTime")
                                   else if (description =="FPSTest"){
                                     param.FPSTest=atoi(charBuf);
                                   }
                                   else if (description =="FPSTestSamples"){
                                     param.FPSTestSamples=atoi(charBuf);
                                   }
                        
                                  //else if (description == "FPSPumpTime"){
                                   // param.FPSPumpTime=atoi(charBuf);
                                  //} //END else if (description == "FPSPumpTime")
                                  //else if (description == "FPSPurgeTime"){
                                  //  param.FPSPurgeTime=atoi(charBuf);
                                 // } //END  else if (description == "FPSPurgeTime")
                                  //else if (description == "delayTime"){
                                  //  param.delayTime=atoi(charBuf);
                                  //} //END else if (description == "delayTime")
                                  else if (description == "FPSCollectTime"){
                                    param.FPSCollectTime=atoi(charBuf);
                                  } //END  else if (description == "FPSCollectTime")
                                  else if (description == "FPSDrainTime"){
                                    param.FPSDrainTime=atoi(charBuf);
                                  } //END else if (description == "FPSDrainTime")
                                  else if (description == "FPSEquilibrateTime"){
                                    param.FPSEquilibrateTime=atoi(charBuf);
                                  } //END else if (description ="FPSEquilibrateTime")
                                  else if (description == "FreshWaterRinseTime"){
                                    param.FreshWaterRinseTime=atoi(charBuf);
                                  } //END else if (description ="FreshWaterRinseTime"
                                  else if (description == "AcidPumpTime"){
                                    param.AcidPumpTime=atoi(charBuf);
                                  } //END else if (description == "AcidPumpTime")
                                  else if (description == "sontekInterval"){
                                    param.sontekInterval=atoi(charBuf);
                                  } //END else if (description == "sontekInterval")
                                  else if (description == "SI"){
                                    param.SI=atoi(charBuf);
                                  } //END else if (description == "SI")
                                  else if (description == "TimeThreshold"){
                                    param.TimeThreshold=atoi(charBuf);
                                  } //END  else if (description == "TimeThreshold")
                                  else if (description == "QThreshold"){
                                    param.QThreshold=atoi(charBuf);
                                  } //END  else if (description == "QThreshold")
                                  else if (description == "TempThreshold"){
                                    param.TempThreshold=atoi(charBuf);
                                  } //END  else if (description == "QThreshold") 
                                  else if (description == "FakeFlow"){
                                    param.FakeFlow=atoi(charBuf);
                                  } //END  else if (description == "QThreshold") 
                                
                                  else{ 
                                    Serial.println("mismatch-look for a typo");
                                  } //END ELSE
 
                                  description="";
                                       }//end   else if(character =='=')
            //reset description
    }//end while (myFile.available())
    
    myFile.close();					// close the file:
    Serial.println("File closed");	// REPORT SUCCESS
    Serial.println("assigned parameters: ");
    
    //siteID=param.SiteID;
    PumpTime=param.PumpTime;
    PumpToProbe=param.PumpToProbe;
    PurgeTime=param.PurgeTime;
    scanCleaningTime=param.scanCleaningTime;
    mantaDelayTime=param.mantaDelayTime;
    FPSCollectTime=param.FPSCollectTime;
    FPSDrainTime=param.FPSDrainTime;
    FPSEquilibrateTime=param.FPSEquilibrateTime;
    FreshWaterRinseTime=param.FreshWaterRinseTime;
    AcidPumpTime=param.AcidPumpTime;
    sontekInterval=param.sontekInterval;
    measurementTime=param.measurementTime;
    FPSTest=param.FPSTest;
    FPSTestSamples=param.FPSTestSamples;
    SI=param.SI;
    TimeThreshold=param.TimeThreshold;
    TempThreshold=param.TempThreshold;
    QThreshold=param.QThreshold;
    FPSOperation=param.FPSOperation;  
    FakeFlow=param.FakeFlow;
  }//end  if(myFile)
  else {
    Serial.println("didn't open!");	//REPORT FAILURE
  } //END else
  Serial.print("pumpTime");  Serial.print(PumpTime);  Serial.println();
  Serial.print(" PumpToProbe");  Serial.print( PumpToProbe);  Serial.println();
  Serial.print("  PurgeTime");  Serial.print(  PurgeTime);  Serial.println();
  Serial.print(" scanCleaningTime");  Serial.print( scanCleaningTime);  Serial.println();
  Serial.print("mantaDelayTime");  Serial.print(mantaDelayTime);  Serial.println();
  Serial.print("measuremetTime");  Serial.print(measurementTime);  Serial.println();
  Serial.print("FPSTest");  Serial.print(FPSTest);  Serial.println();
  Serial.print("FPSTestSamples");  Serial.print(FPSTestSamples);  Serial.println();
  Serial.print(" FPSCollectTime");  Serial.print(FPSCollectTime);  Serial.println();
  Serial.print("FPSDrainTime");  Serial.print(FPSDrainTime);  Serial.println();
  Serial.print("FPSEquilibrateTime");  Serial.print(  FPSEquilibrateTime);  Serial.println();
  Serial.print("FreshWaterRinseTime");  Serial.print(FreshWaterRinseTime);  Serial.println();
  Serial.print("AcidPumpTime");  Serial.print(AcidPumpTime);  Serial.println();
  Serial.print("sontekInterval");  Serial.print(sontekInterval);  Serial.println();
  Serial.print("SI");  Serial.print(SI);  Serial.println();
  Serial.print(" TimeThreshold");  Serial.print( TimeThreshold);  Serial.println();
  Serial.print(" TempThreshold");  Serial.print( TempThreshold);  Serial.println();
  Serial.print(" QThreshold");  Serial.print( QThreshold);  Serial.println();
  Serial.print(" FPSOperation");  Serial.print( FPSOperation);  Serial.println();
  Serial.print(" FakeFlow");  Serial.print( FakeFlow);  Serial.println();
   
  }//end getSettings() FUNCTION
  
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
 
   //DECIDE WHETHER OR NOT SAMPLING IS OK BASED ON READINGS FROM SENSORS
   //THE FLOAT SENSOR REPORTING LOW --BOTTLE FULL
   //ONE TEMP SENSOR REPORTING BELOW FREEZING TEMPS IS A GOOD REASON TO LOOK AT THE TWO 
   //OF THEM TOGETHER. LETS HAVE IT BE A THREE LEVEL THING:
   //--LEVEL 0 ALL CLEAR SAMPLE AWAY
   //LEVEL 1 BOTTLE FULL DON'T TAKE FPS BUT CONTINUE PUMPING ROUTINES
   //LEVEL 2 TOO COLD, DON'T RUN ANY PUMPS, BUT TAKE FLOW MEASUREMENT
   //LEVEL 3 TOO COLD AND BOTTLE FULL--NO DIFFERENCE, JUST EASIER BOOK KEEPING
   //LEVEL 4 INSTRUMENT NOT SUBMERGED
   
 int allClear(){
     int allClear=0;
     int tooCold=0;
     int bottleFull=0;
     
     if (errorCodes[4]){ //the float switch says the collection bottle is full
           bottleFull=1;
           Serial.println("bottle full, don't collect");
         }
     if (errorCodes[3] || errorCodes[2]){//if one of the temp probes says it is COLD;  
          Serial.println("thermal warning on at least one sensor");                      
         //READ THEM BOTH AGAIN                   
         DateTime now = RTC.now();
         sensorValues[2] =now.tempC();               //RTC degreesC
         sensorValues[1] = analogRead(thermistor);   //thermistior raw
         float temp =(1.1*100*sensorValues[1])/1023; //thermistor degreesC
                            if(abs(sensorValues[2]-temp)>5){ 
								//assume the thermistor is being flaky if 
								//they are very different
                                    if (sensorValues[2] <=TempThreshold){ 
                                      Serial.println("too cold to pump");
                                      tooCold=2;     //it is too cold to sample
                                    }//end if (sensorValues[2] <=0) 
                                    else{
                                      Serial.println("just a flaky thermistor");
                                    }        
                                                
                              }//end if(abs(sensorValues[2]-temp)>5)
                           
  
                        }//end if (errorCodes[3] || errorCodes[2])
   
    allClear=tooCold+bottleFull;
     Serial.print("allClear: "); Serial.println(allClear);
   return allClear;
 }//END  int allClear() FUNCTION

/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/

///FUNCTION TO AUTOMATE THE COLLECTION OF FPS IT RETURNS THE NUMBER OF SAMPLES IT SHOULD
///HAVE COLLECTED, WHICH CAN BE DIFFERENT FROM THE NUMBER OF SAMPLES THAT IT WAS ASKED TO
///COLLECT
///IT HAS BEEN PULLED OUT OF THE MAIN LOOP FOR THE SAKE OF DECLUTTERING THE ESSENTIAL 
///FUNCTION OF THE SYSTEM

  int collectFlowProportionalSample(int numberOfSamples){
		int collect=0;
if(digitalRead(floatSwitch)==1){
		unsigned long int counter=millis();
 			if (numberOfSamples<=5){
					collect=numberOfSamples;
				}//end if (numberOfSamples<=5)
			else if (numberOfSamples>5){ 
					//JUST IN CASE SOMETHING CRAZY HAPPENS, WE DON'T WANT TO GET STUCK 
					//COLLECTING SAMPLES INSTEAD OF LISTENING FOR A SAMPLE ROUTINE.
					//ALSO, WE SHOULD DESIGN THIS SO THE THRESHOLD ISN'T ACTUALLY 
					//VIOLATEDBUT FOR NOW 5 SAMPLES SEEMS LIKE A LOT
					collect=5;           
				}//end else if (numberOfSamples>5)
				
///FOR EACH REQUIRED SAMPLE               
           for (int i=1; i<=collect; i++){
					Serial.print("collecting FPS sample(s): ");	//GIVE CONTEXT		
					Serial.println(i);							//REPORT SAMPLE NUMBER
				
///PUMP TO THE COLLECTION VESSEL
					pump(systemPump,FPSCollectTime,FPSCollectValve);
								
///ALLOW COLLECTION VESSEL TO EQUILIBRATE (IE DRAIN EXCESS FROM PORT IN SIDE)
					counter=millis(); //set counter
                    while(millis() - counter < FPSEquilibrateTime*1000){
						//WAIT FOR METERING VESSEL TO DRAIN OFF EXCESS SAMPLE
						} //END while(millis()-counter<FPSEquilibrateTime*1000)
					  
///COLLECT FPS TO COMPOSITE SAMPLE BOTTLE          
                    digitalWrite(FPSCollectDrainValve,LOW); 	//OPEN DRAIN VALVE
					counter=millis();
					while(millis() - counter < FPSDrainTime*1000){
						//WAIT FOR SAMPLE TO DRAIN FROM METERING VESSEL INTO 
						//COMPOSITE COLLECTION VESSEL
						} //END while(millis()-counter<FPSEquilibrateTime*1000)
                     digitalWrite(FPSCollectDrainValve,HIGH);  //CLOSE DRAIN VALVE AGAIN
			   } //END  for (int i=1; i<=collect; i++)
}//end if getFloatSwitch
	return collect;			   
}//end collectFlowProportionalSample(int numberOfSample) FUNCTION



 void DrainAllValves (unsigned long int drainTime, int pumpTime){
   Serial.println("draining Valves for safety and security");
 digitalWrite(ScanDrain, LOW);
 digitalWrite(FPSCollectValve,LOW);
 digitalWrite(WaterQualityValve,LOW);
 pump(systemPump,pumpTime,WaterQualityValve);
 pump(systemPurge,pumpTime+2,WaterQualityValve);
 unsigned long int start=millis();
 while ((millis()-start)<drainTime){
   
 }
 digitalWrite(ScanDrain, HIGH);
 digitalWrite(FPSCollectValve,HIGH);
 digitalWrite(WaterQualityValve,HIGH);
 }//end drainAllValves





/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
///A FUNCTION WRITTENT TO SERIALLY TEST THE OUTPUTS THAT THE ARDUINO IS ABLE TO CONTROL
///THE HEADER MAKES PINS 23 THROUGH 39 AVAILABE
///I USED THIS TO CHECK THE SOLDERING ON I/O CHANNELS I WASN'T USING
///AND SOME THAT I WAS
/*
void testOutputs(){
  for (int i=23; i<39;i++){
    Serial.print("testing output "); Serial.print(i);
    digitalWrite(i,HIGH);
    delay(2000);
    digitalWrite(i,LOW);
    delay(2000);
    Serial.println();
  }
}
  */

  
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/*
void sampleRoutine(){
                 //pump to probe
                          Serial.println("Drain open");
                          Serial.println("Pumping");
                          timeStamp();
                      runPump(systemPump,WaterQualityValve,PumpTime);            
                 //wait for manta
                         Serial.println("Wait for Manta");
                         timeStamp();
                  now=RTC.now();
                  long int wait=(SI*60)-(now.unixtime()%(SI*60))+mantaDelayTime;
                 for(int i=0;i<wait;i++){
                          delay(1000);
                          Serial.print(".");
                     } 
                          Serial.println();
                          timeStamp();
                 //purge from probe
                     runPump(systemPurge,WaterQualityValve,PurgeTime);
                         timeStamp();
                      Serial.println("Waiting... ");
   }//end while timeToSample<5
   
 */  
 
 
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
 ///OLD VERSION OF THE fps ROUTINE BEFORE IT WAS PIGGYBACKED ON THE SCAN SYSTEM
 /*
  void FlowProportionalSample(int duration) {
       digitalWrite(FPSpump,HIGH);
        Serial.println("entering FPS");
       //PURGE LINES 
       Serial.println("fill lines");
             pump(FPSpump,FPSpurgeTime,FPSpurgeValve); //pump to use, duration, and valve 
													   //to use (if no valve, 0)
       //COLLECT WATER SAMPLE
       Serial.println("collect water sample"); 
             pump(FPSpump,duration,FPSpumpValve);
        //EVACUATE PUMP LINES 
        Serial.println("empty lines");         
               pump(FPSpurge,FPSpurgeTime+5,FPSpumpValve);
        Serial.println("leaving FPS");
        digitalWrite(FPSpump,LOW);
     }
    */ 

/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/* ///WAS A NOTIONAL FUNCTION TO PROVIDE NUMBERS RATHER THAN SONTEK READINGS
   ///FOR SYSTEM CALIBRATION AND TESTING
int readSontekValues(DateTime startRoutine){
  int duration=startRoutine.second(); 
		//instead of reading the sontek lets just get a random number
         //next will read it from a file on the SD card so I can test things.
         //write variables to log file
         //store others for access to outside routines
      
        return duration;
}
*/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/
/* ///A NOTIONAL FUNCTION FOR RUNNING A SAMPLE ROUTINE INDEPENDENTLY OF THE MAIN LOOP
///NOT ACTUALLY USED ANYMORE
void pumpRoutine(){
    Serial.println("starting pump Routine");
    
    long int startSystemPumpRoutine=millis();
    long int cycleTime=millis();
    
          //PUMP TO PROBES
          pump(systemPump,PumpTime+PumpToProbe,0);
            delay(100);
    
    //WAIT FOR MANTA AND SCAN TO TAKE MEASUREMENT
          cycleTime=millis();
          Serial.println("delaying");
            while ((millis()-cycleTime)/1000 < (mantaDelayTime+measurementTime)){  
			}
          delay(100);
    
    //EMPTY THE PUMP LINES
           pump(systemPurge,PurgeTime,0);
             delay(100);
    
    Serial.println("leave pumpRoutine");
  
}
*/

/**************************************************************************************/ 
/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/  

/*
void runPump(int pump, int valve, int duration){
  //open valve
        digitalWrite(valve,HIGH);
        delay(100);
  //turn on pump
        digitalWrite(pump,HIGH);
  //delay
        for (int i=0;i<duration;i++){
          delay(1000);
          Serial.print(".");
        } 
        Serial.println();
  //turn pump off
        digitalWrite(pump,LOW);
  //close valve
        digitalWrite(valve,LOW);
} //end runPump() FUNCTION
*/
