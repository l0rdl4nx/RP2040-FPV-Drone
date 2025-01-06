#include "PID.h"
#include <MPU9250.h>
#include <filesystem.h>

// operating mode, for calculation difference take a look at PID.h --> here "Proportional on Measurement" as PONM and "Proportional on Error" as PONE
// generally PONM is for this application more useful since it prevents "overshooting"; PONE is more useful for testing
#define pidMODE PONE

// create the gyro pbject (MPU9250)
MPU9250 mpu;

filesystem filesys;

config *cfg;

float input[4] = {};
float setpoint[4] = {};
float output[4] = {};

// PID-controller for throttle (height) and pitch (up-down), roll (left-right "rolling") and yaw (left-right "turning") axis
PID calcThrottle(&input[THR], &setpoint[THR], &output[THR]);
PID calcPitch(&input[PITCH], &setpoint[PITCH], &output[PITCH]);
PID calcRoll(&input[ROLL], &setpoint[ROLL], &output[ROLL]);
PID calcYaw(&input[YAW], &setpoint[YAW], &output[YAW]);

class stabilization
{
private:
    bool autolevel = true; // Turns the stabilization either ON (true) or OFF (false)
    bool lastAutolevel = false;

    float (*Angles)[];
    float (*Motors)[];

    boolean debug;

    void applyCalibration()
    {
        mpu.setAccBias(cfg->acc[X], cfg->acc[Y], cfg->acc[Z]);
        mpu.setGyroBias(cfg->gyro[X], cfg->gyro[Y], cfg->gyro[Z]);
        mpu.setMagBias(cfg->mag[X], cfg->mag[Y], cfg->mag[Z]);
        mpu.setMagScale(cfg->magScale[X], cfg->magScale[Y], cfg->magScale[Z]);
    }

    void applyConfig()
    {
        calcThrottle.setParams(cfg->throttle[KP], cfg->throttle[KI], cfg->throttle[KD]);
        calcPitch.setParams(cfg->pitch[KP], cfg->pitch[KI], cfg->pitch[KD]);
        calcRoll.setParams(cfg->roll[KP], cfg->roll[KI], cfg->roll[KD]);
        calcYaw.setParams(cfg->yaw[KP], cfg->yaw[KI], cfg->yaw[KD]);

        calcThrottle.setOutputLimits(cfg->throttle[MINOUT], cfg->throttle[MAXOUT]);
        calcPitch.setOutputLimits(cfg->pitch[MINOUT], cfg->pitch[MAXOUT]);
        calcRoll.setOutputLimits(cfg->roll[MINOUT], cfg->roll[MAXOUT]);
        calcYaw.setOutputLimits(cfg->yaw[MINOUT], cfg->yaw[MAXOUT]);
    }

public:
    stabilization(float (*ang)[], float (*motor)[], config *cfggg)
    {
        Angles = ang;
        Motors = motor;
        cfg = cfggg;
    }

    void calibrateMPU()
    {
        mpu.verbose(true);
        delay(5000);
        mpu.calibrateAccelGyro();
        delay(5000);
        mpu.calibrateMag();
        mpu.verbose(false);

        for (int i = 0; i < 3; i++)
            cfg->acc[i] = mpu.getAccBias(i);
        for (int i = 0; i < 3; i++)
            cfg->gyro[i] = mpu.getGyroBias(i);
        for (int i = 0; i < 3; i++)
            cfg->mag[i] = mpu.getMagBias(i);
        for (int i = 0; i < 3; i++)
            cfg->magScale[i] = mpu.getMagScale(i);

        filesys.saveCalibration(cfg);
    }

    /*
     * Starts up connection to MPU9250 on given address (default 0x68), mounts the filesystem.
     *
     * @return true if success, false if it fails: logs error message if 'debug' is set to true
     */
    boolean Start()
    {
        if (!mpu.setup(0x68)) // change if needed
        {
            if (debug)
                Serial.println("Couldn't connect to MPU9250 or no module found on given address, returning.");
            return false;
        }

        if (!filesys.mount())
        {
            if (debug)
                Serial.println("Mount of filesystem failed, returning.");
            return false;
        }

        getMPUAngles();
        int stateCal = filesys.getCalibration(cfg);
        int statePID = filesys.getPID(cfg);

        if (stateCal != SUCCEEDED || statePID != SUCCEEDED)
        {
            if (debug)
            {
                if (stateCal == FAILED_FILE_OPEN || statePID == FAILED_FILE_OPEN)
                {
                    Serial.print("Failed to open the ");
                    Serial.println((stateCal == FAILED_FILE_OPEN) ? "calibration file." : "PID tunings file.");
                }

                if (stateCal == FAILED_JSON || statePID == FAILED_JSON)
                {
                    Serial.print("Failed to deserialize the ");
                    Serial.println((stateCal == FAILED_JSON) ? "calibration file." : "PID tunings file.");
                }
            }

            return false;
        }

        applyCalibration();
        applyConfig();

        calcThrottle.setCalculationMode(pidMODE);
        calcPitch.setCalculationMode(pidMODE);
        calcRoll.setCalculationMode(pidMODE);
        calcYaw.setCalculationMode(pidMODE);

        for (int i = 0; i < 4; i++)
            (*Motors)[i] = 0;

        return true;
    }

    /*
     * Computes the PID correction and adjust the corresponding variables; has to be called every loop().
     */
    void Compute()
    {
        if (!autolevel) // only stabilize if autolevel is on
        {
            if (lastAutolevel = true)
                for (int i = 0; i < 4; i++)
                    (*Motors)[i] = 0;
            lastAutolevel = false;
        }
        else if (mpu.update()) // if new gyro data is available, execute the stabilization update
        {
            input[PITCH] = mpu.getPitch();
            input[ROLL] = mpu.getRoll();
            input[YAW] = mpu.getYaw();
            input[THR] = 0;

            calcThrottle.calculate();
            calcPitch.calculate();
            calcRoll.calculate();
            calcYaw.calculate();

            // (*Motors)[FRONT_LEFT] = output[THR] - output[PITCH] + output[ROLL] + output[YAW];
            // (*Motors)[FRONT_RIGHT] = output[THR] - output[PITCH] - output[ROLL] - output[YAW];
            // (*Motors)[BACK_LEFT] = output[THR] + output[PITCH] + output[ROLL] - output[YAW];
            // (*Motors)[BACK_RIGHT] = output[THR] + output[PITCH] - output[ROLL] + output[YAW];

            (*Motors)[FRONT_LEFT] = +output[PITCH] + output[ROLL];
            (*Motors)[FRONT_RIGHT] = +output[PITCH] - output[ROLL];
            (*Motors)[BACK_LEFT] = -output[PITCH] + output[ROLL];
            (*Motors)[BACK_RIGHT] = -output[PITCH] - output[ROLL];

            lastAutolevel = true;
        }
    }

    void getMPUAngles()
    {
        mpu.update();

        (*Angles)[PITCH] = mpu.getPitch();
        (*Angles)[ROLL] = mpu.getRoll();
        (*Angles)[YAW] = mpu.getYaw();
    }

    /*
     * Set the limits for PID regulation, use for different flying modes etc. Default is -200 for MinOut and 200 for MaxOut.
     *
     * @param usedPid use the defined values for PITCH, ROLL, YAW and THR for telling which PID-controller to change
     * @param min new MinOut value for PID
     * @param max new MaxOut value for PID
     */
    void setLimits(int usedPid, int min, int max)
    {
        switch (usedPid)
        {
        case PITCH:
            cfg->pitch[MINOUT] = min;
            cfg->pitch[MAXOUT] = max;
            break;
        case ROLL:
            cfg->roll[MINOUT] = min;
            cfg->roll[MAXOUT] = max;
            break;
        case YAW:
            cfg->yaw[MINOUT] = min;
            cfg->yaw[MAXOUT] = max;
            break;
        case THR:
            cfg->throttle[MINOUT] = min;
            cfg->throttle[MAXOUT] = max;
            break;
        }

        filesys.savePID(cfg);
    }

    /*
     * Set the PID setpoints as an array. Defined PITCH, ROLL, YAW and THR are used to identify the value indices.
     *
     * @param newSp[4] array of setpoints
     */
    void setSetpoints(float newSp[4])
    {
        setpoint[THR] = newSp[THR];
        setpoint[PITCH] = newSp[PITCH];
        setpoint[ROLL] = newSp[ROLL];
        setpoint[YAW] = newSp[YAW];
    }

    float testMPU()
    {
        mpu.update();
        return mpu.getPitch();
    }

    void formatFS()
    {
        filesys.format();
    }

    void setDebug(boolean dbg)
    {
        filesys.setDebugForFS(dbg);
        debug = dbg;
    }
};