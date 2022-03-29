# BinderMachine2022

BinderMakerMachine_AutoReset-v5.ino

When the selector switch is turned, the process begins with the WATER_METERING case, pumping water into the container until the target weight is reached. The syringe pump primes itself. The next case depends on the batch size. If the batch size is equal to RunLargeBatch the INITIAL_LOWER_STAGE case will execute. The stage lowers the mixer into the container halfway and the mixer turns ON. If the batch size is equal to RunSmallBatch, the INITIAL_LOWER_STAGE case skips. Next, the SUGAR_METERING case dispenses sugar into the container until the target weight is reached. The vibration motor will turn on when the sugarMetering comes close to target weight. The vibration motor shakes out excess sugar in the sugar nozzle. The vibration motor turns off when the target weight is reached. Also in this case and if the batch size is equal to RunLargeBatch, the mixer will turn off when the scale almost reaches target weight. The Proxel dispenses into the container in the BIOCIDE_METERING case. The ADD_MORE_WATER case decides wether or not to add more water to the binder if too much sugar was added. The LOWER_MIXER case lowers the stage into the bottom of the container and the MIXING case mixes the binder. After mixing, the RAISE_MIXER case raises the stage from the container.

At any point in time, the selector switch can be turned to "IDLE" and the process will stop and the stage will reset itself to the top of the machine.

