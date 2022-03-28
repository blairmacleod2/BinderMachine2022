/*3/25/22
   Process: waterMetering, InitialLowerMixer, sugarMetering and mixing,
   Proxel, addMoreWater, LowerMixer, Mix, Raise Mixer.

   this scripts lowers the mixer into the container after the water is first added.
   It begins mixing while the sugar is dispensing.
*/


#include "AccelStepper.h"
#include "pins_RUMBA.h"
#include "ScaleEJSeries.h"
#include "ezButton.h"

//start and stop exiting variables
bool x = true;  //used for the mixer and stage
bool title = true;
bool resetState = true;
int vibratorSpeed = 80;
float waterAveOvershoot = 4;  //2.4 kept producing 21.1% concentrations

//for two selection switch, two batch sizes
float smallBatchSize = 1000.0;
float largeBatchSize = 4000.0;
float selectedBatchSize;

float smallMixingTime = 180000; //3 minutes
float largeMixingTime = 300000; //5 minutes
float mixingTime; //Added

//Master recipe ratios. must add up to 1000g.
float biocidePerKg = 0.6;  //0.6ml of the water in each liter replaced with dilute proxel solution
float sugarPerKg = 212.4;  //21% sugar by mass (21.2% sugar)
float waterPerKg = 787.0;  //remainder water

//These numbers will be calculated by updateRecipe(batchSize) before beginning
//and then recalculated to match actual water quantity following Water Metering
float biocideVolume;      //undiluted 0.175ml proxel. 1:11 proxel:water dilution ratio. 2.1ml pre-diluted proxel solution
float targetWaterWeight;  //water target 2765g - 2.0ml biocide volume
float targetSugarWeight;  //sugar mass
float targetAddedWaterWeight;

//variables to keep track of actual measured quantities
float deliveredWaterWeight;
float deliveredSugarWeight;
float deliveredBinderWeight;
float deliveredConcentration;
float deliveredAddedWaterWeight;

//Variables for tracking cycle times
unsigned long cycleStartTime, cycleFinishTime;

int waterPumpPin1 = FAN1_PIN;   //water pumps
int waterPumpPin2 = FAN0_PIN;
int pumpOutputPin = TEMP_2_PIN; //syringe pump "Busy" signal
int selectorPin1 = Z_MAX_PIN;     //switch
int selectorPin2 = Z_MIN_PIN;     //switch
int selectorIndicatorPin = HEATER_0_PIN; //indicator light on switch
int mixer = HEATER_1_PIN; //mixer
int vibrator = HEATER_2_PIN;

enum moveType {
  PICKUP,
  DISPENSE
};

enum selection {   //enum for passing selector state
  RunSmallBatch,
  StopMachine,
  RunLargeBatch,
  SelectorError,
};

enum State {
  IDLE_STATE,
  WATER_METERING,
  DELAY_AFTER_WATER,
  INITIAL_LOWER_MIXER,
  SUGAR_METERING,
  DELAY_AFTER_SUGAR,
  BIOCIDE_METERING,
  DELAY_AFTER_BIOCIDE,
  ADD_MORE_WATER,
  DELAY_AFTER_WATER_ADD,
  LOWER_MIXER,
  DELAY_BEFORE_MIXING,
  MIXING,
  DELAY_AFTER_MIXING,
  RAISE_MIXER,
  EVALUATE_RESULTS
};

//sugar metering global variables
int topAugerSpeed = 1300;  //2000
int slowAugerSpeed = 30;  //40
float startingSugarWeight;
float sugarWeight;

//water metering global variables
int fastWaterPumpDuty = 255;
int slowWaterPumpDuty = 160;
float startingWaterWeight;
float waterWeight;
float startingAddWaterWeight;

//Syringe pump variables and assignments
const int pumpAddress = 2;      //valve assignments
const int valvePort_Output = 5;
const int valvePort_Proxel = 6;
const int valvePort_Air = 4;
const int speed_PickupAir = 1500;
const int speed_DispenseAir = 1500;
const int speed_PickupProxel = 800;
const int speed_DispenseProxel = 800;
const float syringeMaxVolume = 2.5;
int syringePosition = 0;
int currentPort = 6;    //presently selected valve port

//scale variables and assignments
float scaleWeight = 0;
float prevScaleWeight;
bool newScaleData = false;
Scale binderScale;      //A&D scale, RS232 interface

//steppers
AccelStepper auger(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);  //auger stepper
AccelStepper stage(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);  //stage stepper
#define motorInterfaceType 1
const long retractDistance = -1600;

//linear stage and limit switches
#define MAX_POSITION 0x7FFFFFFF // maximum of position we can set (long type)
#define DIRECTION_CCW -1
#define DIRECTION_CW   1
bool isStopped = false;
//int direction  = DIRECTION_CW;
long targetPos = 0;
ezButton limitSwitch_Bottom(TEMP_1_PIN);
ezButton limitSwitch_Top(TEMP_0_PIN);

State machineState = IDLE_STATE;
State nextMachineState = IDLE_STATE;
bool stateEntry = true;

void setup()
{
  Serial.begin(115200);   //USB
  Serial3.begin(9600);    //syringepump
  Serial2.begin(9600);    //scale

  //syringe pump and water pumps
  pinMode(pumpOutputPin, INPUT);  //syringe pump
  pinMode(waterPumpPin1, OUTPUT);
  pinMode(waterPumpPin2, OUTPUT);
  digitalWrite(waterPumpPin1, LOW);
  digitalWrite(waterPumpPin2, LOW);

  //selector switch for IDLE, 4Liters, and 1 Liter
  pinMode(selectorIndicatorPin, OUTPUT);
  pinMode(selectorPin1, INPUT);
  pinMode(selectorPin2, INPUT);
  digitalWrite(selectorIndicatorPin, LOW);
  digitalWrite(selectorPin1, HIGH);     //set pullup
  digitalWrite(selectorPin2, HIGH);

  //Auger Stepper
  pinMode(Y_ENABLE_PIN, OUTPUT);
  digitalWrite(Y_ENABLE_PIN, HIGH);
  auger.setMaxSpeed(4000);
  auger.setAcceleration(8000);
  auger.enableOutputs();
  auger.setCurrentPosition(0);
  auger.setPinsInverted(true, false, false);

  //linear stage stepper
  pinMode(Z_ENABLE_PIN, OUTPUT);  //steppper
  digitalWrite(Z_ENABLE_PIN, HIGH);
  stage.enableOutputs();
  stage.setMaxSpeed(1000.0);   // set the maximum speed
  stage.setAcceleration(800.0); // set acceleration
  stage.setSpeed(250);         // set initial speed
  stage.setCurrentPosition(0); // set position
  stage.setPinsInverted(false, false, false);

  //limit switches
  limitSwitch_Bottom.setDebounceTime(50); // set debounce time to 50 milliseconds
  limitSwitch_Top.setDebounceTime(50); // set debounce time to 50 milliseconds

  //mixer
  pinMode(mixer, OUTPUT);
  digitalWrite(mixer, LOW); //stop mixer
  //vibrator
  pinMode(vibrator, OUTPUT);
  digitalWrite(vibrator, LOW);
  initPump();
}


void loop()
{
  weirdMixerIssue(); //this keeps the stage from changing directions (don't know why)
  readScale();
  stateMachine();
}


void stateMachine()
{
  if (readSelector() == StopMachine)
  {
    nextMachineState = IDLE_STATE; //always allow return to IDLE state
  }
  if (machineState != nextMachineState)
  {
    if ((nextMachineState == IDLE_STATE) && (machineState != EVALUATE_RESULTS))
    { digitalWrite(selectorIndicatorPin, HIGH);
      Serial.println();
      Serial.println("Machine Stopped!   Resetting Machine");
      Serial.println(); Serial.println();
      stage.setCurrentPosition(0);
      stage.moveTo(0);
      stage.runToPosition();
      digitalWrite(vibrator, LOW);
      digitalWrite(mixer, LOW);
      analogWrite(waterPumpPin1, 0);
      analogWrite(waterPumpPin2, 0);
      delay(1000);
    }
    machineState = nextMachineState;
    stateEntry = true;
    x = true; //linear stage and mixer
    title = true;
    resetState = true;
  }

  switch (machineState)
  {
    case IDLE_STATE:
      {
        digitalWrite(vibrator, LOW);
        digitalWrite(mixer, LOW);
        analogWrite(waterPumpPin1, 0);
        analogWrite(waterPumpPin2, 0);
        if (stateEntry)
        {
          if (readSelector() == StopMachine)
          {
            if (title)
            {
              Serial.println("------3DEO Binder Making Machine 2022------");
              Serial.println();
              Serial.println("Entered Idle State");
              Serial.println();
              title = false;
            }
            if (resetStage(x))
            {
              digitalWrite(selectorIndicatorPin, LOW);
              stateEntry = false;
            }
          }
          else
          {
            blinkIndicator();
          }
        }
        else
        {
          digitalWrite(selectorIndicatorPin, LOW);
          if ((readSelector() !=  StopMachine) && (readSelector() !=  SelectorError))
          {
            if (readSelector() ==  RunSmallBatch)
            {
              Serial.println("Entering Run State, Small batch.");
              mixingTime = smallMixingTime;
              selectedBatchSize = smallBatchSize;
              updateRecipe(selectedBatchSize, mixingTime);
            }
            else if (readSelector() ==  RunLargeBatch)
            {
              Serial.println("Entering Run State, Large batch.");
              mixingTime = largeMixingTime;
              selectedBatchSize = largeBatchSize;
              updateRecipe(selectedBatchSize, mixingTime);
            }
            nextMachineState = WATER_METERING;
            //            nextMachineState = LOWER_MIXER;
          }
        }
      }
      break;


    case WATER_METERING:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        if (stateEntry)
        {
          waitForScaleUpdate();
          cycleStartTime = millis();
          startingWaterWeight = scaleWeight;
          stateEntry = false;
          Serial.println("Entering Water Metering State");
          Serial.print("Target Water weight: ");
          Serial.print(targetWaterWeight);
          Serial.println("g");
        }
        else
        {
          if (waterMetering(targetWaterWeight))
          {
            nextMachineState = DELAY_AFTER_WATER;
          }
          else
          {
            nextMachineState = WATER_METERING;
          }
        }
      }
      break;


    case DELAY_AFTER_WATER:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry)
        {
          stateEntry = false;
          stateEntryTime = millis();
        }
        else
        {
          if ((millis() - stateEntryTime) > 3000)
          {
            waitForScaleUpdate();
            Serial.print("Actually delivered ");
            deliveredWaterWeight = scaleWeight - startingWaterWeight;
            Serial.print(deliveredWaterWeight);
            Serial.println("g of water.");
            float newBatchSize = (deliveredWaterWeight / waterPerKg) * 1000.0;
            updateRecipe(newBatchSize, mixingTime);
            primeSyringe();
            //            nextMachineState = SUGAR_METERING;

            //create an if statement that will skip the INITIAL_LOWER_MIXER and go straight to SUGAR_METERING when 1liter is selected or if the weight of the scale is less than 1500g

            if (readSelector() ==  RunSmallBatch)
            {
              nextMachineState = SUGAR_METERING;
            }
            if (readSelector() == RunLargeBatch)
            {
              nextMachineState = INITIAL_LOWER_MIXER;
            }
          }
        }
      }
      break;

    case INITIAL_LOWER_MIXER:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry) //starts as true
        {
          stateEntry = false;
          stateEntryTime = millis();
        }
        else
        {
          if (initialLowerStage(x))
          {
            nextMachineState = SUGAR_METERING;
          }
          else
          {
            nextMachineState = INITIAL_LOWER_MIXER;
          }
        }
      }
      break;


    case SUGAR_METERING:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        if (stateEntry)
        {
          waitForScaleUpdate();
          startingSugarWeight = scaleWeight;
          Serial.println("Entering Sugar Metering State");
          Serial.print("Target Sugar weight: ");
          Serial.print(targetSugarWeight);
          Serial.println("g");
          rampAuger(topAugerSpeed);
          stateEntry = false;
        }
        else
        {
          if (sugarMetering(targetSugarWeight))
          {
            digitalWrite(vibrator, LOW);
            digitalWrite(mixer, LOW);
            nextMachineState = DELAY_AFTER_SUGAR;
          }
          else
          {
            nextMachineState = SUGAR_METERING;
          }
        }
      }
      break;


    case DELAY_AFTER_SUGAR:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry)
        {
          stateEntry = false;
          stateEntryTime = millis();
          //retractAuger(retractDistance);
        }
        else
        {
          if ((millis() - stateEntryTime) > 3000)
          {
            waitForScaleUpdate();
            Serial.print("Actually delivered ");
            deliveredSugarWeight = scaleWeight - startingSugarWeight;
            Serial.print(deliveredSugarWeight);
            Serial.println("g of sugar.");
            Serial.println();
            nextMachineState = BIOCIDE_METERING;
          }
        }
      }
      break;


    case BIOCIDE_METERING:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        Serial.println("Entering Preservative Metering State");
        stateEntry = false;
        doseBiocide(biocideVolume);
        nextMachineState = DELAY_AFTER_BIOCIDE;
      }
      break;


    case DELAY_AFTER_BIOCIDE:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry)
        {
          stateEntry = false;
          stateEntryTime = millis();
        }
        else
        {
          if ((millis() - stateEntryTime) > 2000)
          {
            if (deliveredSugarWeight > targetSugarWeight)
            {
              float newBatchSize = (deliveredSugarWeight / sugarPerKg) * 1000.0;
              updateRecipe(newBatchSize, mixingTime);
              nextMachineState = ADD_MORE_WATER;
            }
            else
            {
              nextMachineState = LOWER_MIXER;
            }
          }
        }
      }
      break;


    case ADD_MORE_WATER:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry)
        {
          waitForScaleUpdate();
          cycleStartTime = millis();
          startingWaterWeight  = scaleWeight;
          targetAddedWaterWeight =  targetWaterWeight - deliveredWaterWeight - waterAveOvershoot;
          stateEntry = false;
          Serial.println("Entering Add More Water state");
        }
        else
        {
          if (waterMetering(targetAddedWaterWeight))
          {
            nextMachineState = DELAY_AFTER_WATER_ADD;
          }
          else
          {
            nextMachineState = ADD_MORE_WATER;
          }
        }
      }
      break;


    case DELAY_AFTER_WATER_ADD:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry)
        {
          stateEntry = false;
          stateEntryTime = millis();
        }
        else
        {
          if ((millis() - stateEntryTime) > 2000)
          {
            waitForScaleUpdate();
            deliveredAddedWaterWeight = scaleWeight - startingWaterWeight;
            deliveredWaterWeight = deliveredAddedWaterWeight + deliveredWaterWeight;
            Serial.print("Target Total Water weight:  ");
            Serial.print(targetWaterWeight);
            Serial.println("g of water.");
            Serial.print("Actually total delivered water:  ");
            Serial.print(deliveredWaterWeight);
            Serial.println("g of water.");
            nextMachineState = LOWER_MIXER;
          }
        }
      }
      break;


    case LOWER_MIXER:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry) //starts as true
        {
          stateEntry = false;
          stateEntryTime = millis();
        }
        else
        {
          if (lowerStage(x))
          {
            nextMachineState = DELAY_BEFORE_MIXING;
          }
          else
          {
            nextMachineState = LOWER_MIXER;
          }
        }
      }
      break;


    case DELAY_BEFORE_MIXING:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        stage.setCurrentPosition(0);
        stage.moveTo(0);
        stage.runToPosition();
        if (stateEntry)
        {
          stateEntry = false;
          stateEntryTime = millis();
        }
        else
        {
          if ((millis() - stateEntryTime) > 1000) //pause for 1 seconds
          {
            nextMachineState = MIXING;
          }
        }
      }
      break;


    case MIXING:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry)
        {
          stateEntry = false;
          stateEntryTime = millis();
        }
        else
        {
          if ((millis() - stateEntryTime) > mixingTime)
          {
            digitalWrite(mixer, LOW);
            nextMachineState = DELAY_AFTER_MIXING;
          }
          else
          {
            digitalWrite(mixer, HIGH);
            nextMachineState = MIXING;
          }
        }
      }
      break;


    case DELAY_AFTER_MIXING:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry)
        {
          stateEntry = false;
          stateEntryTime = millis();
        }
        else
        {
          if ((millis() - stateEntryTime) > 1000) //pause for 1 seconds
          {
            nextMachineState = RAISE_MIXER;
          }
        }
      }
      break;


    case RAISE_MIXER:
      {
        digitalWrite(selectorIndicatorPin, HIGH);
        static unsigned long stateEntryTime;
        if (stateEntry) //starts as true
        {
          stateEntry = false;
          stateEntryTime = millis();
        }
        else
        {
          if (raiseStage(x))
          {
            nextMachineState = EVALUATE_RESULTS;
          }
          else
          {
            nextMachineState = RAISE_MIXER;
          }
        }
      }
      break;


    case EVALUATE_RESULTS:
      {
        stage.setCurrentPosition(0);
        stage.moveTo(0);
        stage.runToPosition();
        cycleFinishTime = millis();
        float cycleTime = (float)(cycleFinishTime - cycleStartTime) / 1000.0;
        stateEntry = false;
        Serial.println(" ");
        Serial.println("Binder Metering Cycle Complete!");
        //Serial.print("Cycle time: ");
        // Serial.print(cycleTime, 3);
        // Serial.println(" seconds");
        Serial.print("Total binder weight delivered: ");
        float total = deliveredWaterWeight + deliveredSugarWeight + biocideVolume;
        Serial.print(total);
        Serial.println("g");

        Serial.print("Actual Delivered Quantity Water: ");
        Serial.print(deliveredWaterWeight);
        Serial.print("g, error: ");
        float waterError = (((deliveredWaterWeight / total) / (waterPerKg / 1000)) - 1.0) * 100.0;
        Serial.print(waterError, 4);
        Serial.println("%");

        Serial.print("Actual Delivered Quantity Sugar: ");
        Serial.print(deliveredSugarWeight);
        Serial.print("g, error: ");
        float sugarError = (((deliveredSugarWeight / total) / (sugarPerKg / 1000)) - 1.0) * 100.0;
        Serial.print(sugarError, 4);
        Serial.println("%");

        Serial.print("Actual Delivered Quantity Preservative: ");
        Serial.print(biocideVolume);
        Serial.println("ml");

        deliveredConcentration = (deliveredSugarWeight / total) * 100;
        Serial.print("Calculated Sugar Concentration: ");
        Serial.print(deliveredConcentration, 4);
        Serial.print("%, error: ");
        float concentrationError = (((deliveredConcentration / 100) / (sugarPerKg / (sugarPerKg + waterPerKg + biocidePerKg))) - 1.0) * 100.0;
        Serial.print(concentrationError, 4);
        Serial.println("%");
        delay(200);
        nextMachineState = IDLE_STATE;
      }
      break;
    default:
      break;
  }
}

bool sugarMetering(float target)
{
  auger.runSpeed();
  if (newScaleData)
  {
    newScaleData = false;
    sugarWeight = scaleWeight - startingSugarWeight;
    float sugarRemaining = target - sugarWeight; //(target + startingSugarWeight) - scaleWeight;
    long newspeed = map(sugarWeight, target - 25.0, target - 8.0, topAugerSpeed, slowAugerSpeed);
    if (newspeed > topAugerSpeed)
    {
      newspeed = topAugerSpeed;
    }
    if (newspeed < slowAugerSpeed)
    {
      newspeed = slowAugerSpeed;
    }
    auger.setSpeed(newspeed);

    if (readSelector() == RunLargeBatch) //starts the mixer during a large batch
    {
      if (0 < sugarRemaining <= target * 0.01)
      {
        digitalWrite(mixer, LOW);
      }
      if (sugarRemaining > target * 0.01)
      {
        digitalWrite(mixer, HIGH);
      }
    }
    if (readSelector() == RunSmallBatch) //stops the mixer during a small batch
    {
      digitalWrite(mixer, LOW);
    }
    
    if (0 < sugarRemaining <= target * 0.01)
    {
      analogWrite(vibrator, vibratorSpeed);
    }
    if (sugarRemaining > target * 0.01)
    {
      digitalWrite(vibrator, LOW);
    }
    if (sugarRemaining <= 0)
    {
      digitalWrite(vibrator, LOW);
      digitalWrite(mixer, LOW);
      rampAuger(0);
      return true;
    }
    else return false;
  }
}

bool waterMetering(float target)
{
  if (newScaleData)
  {
    newScaleData = false;
    waterWeight = scaleWeight - startingWaterWeight;
    float waterRemaining = (target + startingWaterWeight) - scaleWeight;
    if (waterRemaining > 100)
    {
      analogWrite(waterPumpPin1, 255);
      analogWrite(waterPumpPin2, 255);
    }
    else if (waterRemaining > 20)
    {
      analogWrite(waterPumpPin1, 180);
      analogWrite(waterPumpPin2, 0);
    }
    else if (waterRemaining > 0 )
    {
      pumpPulse();
    }
    if (waterRemaining <= 0 )
    {
      analogWrite(waterPumpPin1, 0);
      analogWrite(waterPumpPin2, 0);
      return true;
    }
    else return false;
  }
}

void updateRecipe(float batchSize, float mixingTime)
{
  Serial.println();
  constrain(batchSize, 0, 5500.0);   //6kg scale maximum
  Serial.print("Desired Batch Size: ");
  Serial.print(batchSize);
  Serial.println("g");
  batchSize = batchSize / 1000;
  biocideVolume = batchSize * biocidePerKg;
  targetSugarWeight = batchSize * sugarPerKg;
  targetWaterWeight = batchSize * waterPerKg;
  Serial.print("Calculated Target - Water: ");
  Serial.print(targetWaterWeight);
  Serial.println("g");
  Serial.print("Calculated Target - Sugar: ");
  Serial.print(targetSugarWeight);
  Serial.println("g");
  Serial.print("Calculated Target - Preservative: ");
  Serial.print(biocideVolume);
  Serial.println("g");
  //  Serial.println();
  Serial.print("Mixing Time: ");
  Serial.print(mixingTime / 60000, 0);
  Serial.println(" minutes");
  Serial.println();
}

void pumpPulse()
{
  const unsigned long pulsePeriod = 3000;  //increased from 1000
  static unsigned long pulseStartTime = 0;
  static bool pulse = false;
  static bool pulseEnded = false;
  //if (duty < 20) duty = 20;
  //if (duty > slowWaterPumpDuty) duty = slowWaterPumpDuty;
  //unsigned long int onTime = map(duty, 0, 255, 1000000, 0);
  unsigned long int onTime = 100;  //changed from 50
  unsigned long pos = millis() - pulseStartTime;
  if (pos > pulsePeriod)
  {
    pulseEnded = true;
    pulse = true;
    pulseStartTime = millis();
  }
  else if (pos > onTime)
  {
    pulse = false;
  }

  if (pulse)
  {
    analogWrite(waterPumpPin1, 100); //changed from 255
  }
  else
  {
    analogWrite(waterPumpPin1, 0);
  }
  analogWrite(waterPumpPin2, 0);
}

void rampAuger(float desiredSpeed)
{
  unsigned long int lastIncrementTime = millis();
  const unsigned long incrementRate = 5;  //10
  const float incrementStep = 20; //50
  float startSpeed = auger.speed();
  float speedRamp = startSpeed;
  bool done = false;
  while (!done)
  {
    if (millis() - lastIncrementTime >= incrementRate)
    {
      lastIncrementTime = millis();
      if (desiredSpeed > startSpeed)
      {
        speedRamp += incrementStep;
        if (speedRamp > desiredSpeed) speedRamp = desiredSpeed;
      }
      else if (desiredSpeed < startSpeed)
      {
        speedRamp -= incrementStep;
        if (speedRamp < desiredSpeed) speedRamp = desiredSpeed;
      }
      if (speedRamp == desiredSpeed) done = true;
      auger.setSpeed(speedRamp);
    }
    auger.runSpeed();
  }
}

void retractAuger(long distance)
{
  unsigned long startTime = millis();
  auger.setMaxSpeed(500);
  const unsigned long timeout = 10000;
  auger.setCurrentPosition(0);
  auger.move(distance);
  while ((auger.distanceToGo() != 0) && (millis() - startTime < timeout )) auger.run();
  auger.setMaxSpeed(4000);
}

void primeSyringe()
{
  sendPumpAddress();
  pumpMove(PICKUP, valvePort_Proxel, 2.0, speed_PickupProxel, 1000);
  pumpMove(DISPENSE, valvePort_Proxel, 2.0, speed_DispenseProxel, 1000);
  pumpMove(PICKUP, valvePort_Proxel, biocideVolume, speed_PickupProxel, 1000);
  executePumpCommand(0);
}

void doseBiocide(float biocideVolume)
{
  /* process:
     pick up full syringe proxel
     dispense full syringe back to proxel port
     pick up dose amount of proxel
     dispense dose to output
     pick up 2ml air
     dispense 2ml air to output */
  Serial.print("Preservative volume: ");
  Serial.print(biocideVolume);
  Serial.println("ml");
  sendPumpAddress();
  pumpMove(DISPENSE, valvePort_Output, biocideVolume, speed_DispenseProxel, 1000);
  pumpMove(PICKUP, valvePort_Air, 2.0, speed_PickupAir, 0);
  pumpMove(DISPENSE, valvePort_Output, 2.0, speed_DispenseAir, 0);
  executePumpCommand(60000);
}

void readScale()
{
  binderScale.receiveData();
  if (binderScale.newData)
  {
    binderScale.newData = 0;
    newScaleData = true;
    prevScaleWeight = scaleWeight;
    scaleWeight = binderScale.measureData;
  }
}

void waitForScaleUpdate()
{
  newScaleData = false;
  while (newScaleData == false) readScale();
}

void initPump()
{
  Serial3.println("/2 J0 k5 Z N1 L1 J7 R");
  currentPort = 6;
  syringePosition = 0;
  delay(50);
}

void moveValve(int port)
{
  const int valveAngles[] = {64, 38, 45, 38, 64, 111};  //the degrees to the next clockwise valve port
  int clockwiseRelativeAngle = 0;
  int clockwiseRelativeSteps = 0;
  clockwiseRelativeSteps = ((port + 6) - currentPort) % 6;
  for (int i = 0; i < clockwiseRelativeSteps; i++)
  {
    clockwiseRelativeAngle += valveAngles[((currentPort - 1) + i) % 6];
  }
  if (clockwiseRelativeAngle < 180) Serial3.print('I');
  else Serial3.print('O');
  Serial3.print(port);
  currentPort = port;
}

void pickup(float vol)
{
  int counts = volumeToCounts(vol);
  if ((syringePosition + counts) <= 24000)
  {
    syringePosition += counts;
    Serial3.print('A');
    Serial3.print(syringePosition);
  }
  else Serial.print("error - pickup commanded beyond syringe upper limit");
}

void dispense(float vol)
{
  int counts = volumeToCounts(vol);

  if ((syringePosition - counts) >= 0)
  {
    syringePosition -= counts;
    Serial3.print('A');
    Serial3.print(syringePosition);
  }
  else Serial.print("error - dispense commanded beyond syringe lower limit");
}

void pumpMove(moveType pumpMoveType, int port, float vol, int velocity, int delayTime)
{
  moveValve(port);
  setPlungerSpeed(velocity);
  if (pumpMoveType == PICKUP) pickup(vol);
  else if (pumpMoveType == DISPENSE) dispense(vol);
  if (delayTime > 0) pumpDelay(delayTime);
}

void pumpDelay(int ms)
{
  Serial3.print("M");
  Serial3.print(ms);
}

int volumeToCounts(float vol)
{
  return (int)((vol / syringeMaxVolume) * 24000);
}

void setPlungerSpeed(int syringeSpeed)
{
  Serial3.print('V');
  Serial3.print(syringeSpeed);
}

void sendPumpAddress()
{
  Serial3.print('/');
  Serial3.print(pumpAddress);
  Serial3.print("J0");
}

void executePumpCommand(unsigned long timeOut)
{
  while (digitalRead(pumpOutputPin) == 0);
  Serial3.print("J7 ");
  Serial3.println('R');
  unsigned long commandStartTime = millis();
  delay(100);
  if (digitalRead(pumpOutputPin)) while (((millis() - commandStartTime) < timeOut) && (digitalRead(pumpOutputPin) == 1));
  while ( millis() - commandStartTime < 100);
  while (((millis() - commandStartTime) < timeOut) && (digitalRead(pumpOutputPin) == 0) && (readSelector() != StopMachine));
}

enum selection readSelector()
{
  selection SelectorPosition;
  bool switchOne = !digitalRead(selectorPin1);
  bool switchTwo = !digitalRead(selectorPin2);
  if ((switchOne == false) && (switchTwo == false))  //both switches open. knob in center.
  {
    digitalWrite(selectorIndicatorPin, LOW);
    SelectorPosition = StopMachine;
  }
  else if ((switchOne == true) && (switchTwo == false)) //left switch closed. knob points right
  {
    SelectorPosition = RunLargeBatch;
  }
  else if ((switchOne == false) && (switchTwo == true)) //right switch closed. knob points left
  {
    SelectorPosition = RunSmallBatch;
  }
  else
  {
    SelectorPosition = SelectorError;  //both switches closed somehow? error.
  }
  return SelectorPosition;
}

void blinkIndicator()
{
  static unsigned long lastToggleTime = 0;
  if ((millis() - lastToggleTime) > 500)
  {
    digitalWrite(selectorIndicatorPin, !digitalRead(selectorIndicatorPin));
    lastToggleTime = millis();
  }
}



bool initialLowerStage(float target)
{
  limitSwitch_Bottom.loop(); // MUST call the loop() function first
  limitSwitch_Top.loop(); // MUST call the loop() function first
  targetPos = DIRECTION_CCW * MAX_POSITION;
  stage.moveTo(targetPos);
  stage.run(); // MUST be called in loop() function
  if (limitSwitch_Bottom.getState() == LOW) // bottom switch
  {
    targetPos = DIRECTION_CW * 1600; //here
    stage.setCurrentPosition(0);
    stage.moveTo(targetPos);
    stage.runToPosition();

    targetPos = DIRECTION_CCW * 100; //here
    stage.setCurrentPosition(0);
    stage.moveTo(targetPos);
    stage.runToPosition();

    isStopped = true;
    return true;
  }
  return false;
  if (isStopped == false)
  {
    if (stage.distanceToGo() == 0)
    {
      stage.setCurrentPosition(0);
      stage.moveTo(targetPos);
    }
    stage.run(); // MUST be called in loop() function
  }
}



bool lowerStage(float target)
{
  limitSwitch_Bottom.loop(); // MUST call the loop() function first
  limitSwitch_Top.loop(); // MUST call the loop() function first
  targetPos = DIRECTION_CCW * MAX_POSITION;
  stage.moveTo(targetPos);
  stage.run(); // MUST be called in loop() function
  if (limitSwitch_Bottom.getState() == LOW) // bottom switch
  {
    targetPos = DIRECTION_CW * 160;
    stage.setCurrentPosition(0);
    stage.moveTo(targetPos);
    stage.runToPosition();
    isStopped = true;
    return true;
  }
  return false;
  if (isStopped == false)
  {
    if (stage.distanceToGo() == 0)
    {
      stage.setCurrentPosition(0);
      stage.moveTo(targetPos);
    }
    stage.run(); // MUST be called in loop() function
  }
}

bool raiseStage(float target)
{
  limitSwitch_Bottom.loop(); // MUST call the loop() function first
  limitSwitch_Top.loop(); // MUST call the loop() function first
  targetPos = DIRECTION_CW * MAX_POSITION;
  stage.moveTo(targetPos);
  stage.run(); // MUST be called in loop() function
  if (limitSwitch_Top.getState() == LOW) //top switch pushed
  {
    targetPos = DIRECTION_CCW * 300;
    stage.setCurrentPosition(0);
    stage.moveTo(targetPos);
    stage.runToPosition();
    isStopped = true;
    return true;
  }
  return false;
  if (isStopped == false)
  {
    if (stage.distanceToGo() == 0)
    {
      stage.setCurrentPosition(0);
      stage.moveTo(targetPos);
    }
    stage.run(); // MUST be called in loop() function
  }
}

bool resetStage(float target)
{
  limitSwitch_Bottom.loop(); // MUST call the loop() function first
  limitSwitch_Top.loop(); // MUST call the loop() function first
  targetPos = DIRECTION_CW * MAX_POSITION;
  stage.moveTo(targetPos);
  stage.run(); // MUST be called in loop() function
  digitalWrite(selectorIndicatorPin, HIGH);
  if (limitSwitch_Top.getState() == LOW) //if top switch is being held down
  {
    targetPos = DIRECTION_CCW * 350;
    stage.setCurrentPosition(0);
    stage.moveTo(targetPos);
    stage.runToPosition();
    resetState = false;
    return true;
  }
  return false;
  if (isStopped == false)
  {
    if (stage.distanceToGo() == 0)
    {
      stage.setCurrentPosition(0);
      stage.moveTo(targetPos);
    }
    stage.run();
  }
}

void weirdMixerIssue()
{
  limitSwitch_Bottom.loop(); // MUST call the loop() function first
  limitSwitch_Top.loop(); // MUST call the loop() function first
}
