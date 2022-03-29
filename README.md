# BinderMachine2022

The BinderMakerMachine_AutoReset-v5
This sketch 
...
   This script begins with WaterMetering. After, if the batch size is equal to RunLargeBatch
   the stage will lower into the container and the mixer will turn ON. If the batch size is 
   equal to RunSmallBatch, the stage will not lower and the mixer will remain OFF. The
   sugarMetering case begins. If the batch size is equal to RunLargeBatch, the mixer will turn
   off when the sugarMetering comes close to target weight. When the target weight is almost reached,
   the vibration motor will turn ON and vibrate the sugar nozzle to shake out excess sugar. The
   vibrator motor turns off when the target weight is reached. 
