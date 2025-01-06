class PID
{
private:
    float Kp, Ki, Kd;

    float Input, Output, SetPoint;
    float error, lastInput;
    float piTerm, dInput;
    float minOutput = 0;
    float maxOutput = 255;

    int sampleTime = 100;

    unsigned long elapsedTime, lastTime;
    bool AutoMode = false;

    void initialize()
    {
        lastInput = *input;
        piTerm = *output;
        if (piTerm > maxOutput)
            piTerm = maxOutput;
        else if (piTerm < minOutput)
            piTerm = minOutput;
    }

public:
#define AUTOMATIC 0
#define MANUAL 1
#define PONE 0
#define PONM 1

    bool opMode = 0;
    bool pMode = 0;

    float *setPoint, *input, *output;

    PID(float *myInput, float *mySetPoint, float *myOutput)
    {
        input = myInput;
        setPoint = mySetPoint;
        output = myOutput;
    }

    PID(float *myInput, float *mySetPoint, float *myOutput, float newKp, float newKi, float newKd)
    {
        input = myInput;
        setPoint = mySetPoint;
        output = myOutput;

        double TimeInSec = (((double)sampleTime) / 1000) / 1000;
        Kp = newKp;
        Ki = newKi * TimeInSec;
        Kd = newKd / TimeInSec;
    }

    PID(float *myInput, float *mySetPoint, float *myOutput, float newKp, float newKi, float newKd, bool mode)
    {
        input = myInput;
        setPoint = mySetPoint;
        output = myOutput;

        double TimeInSec = (((double)sampleTime) / 1000) / 1000;
        Kp = newKp;
        Ki = newKi * TimeInSec;
        Kd = newKd / TimeInSec;

        pMode = mode;
    }

    /*
     * Set the new calculating parameters
     *
     * @param Kp
     * @param Ki integral
     * @param Kd
     */
    void setParams(float kp, float ki, float kd)
    {
        double TimeInSec = (((double)sampleTime) / 1000) / 1000;
        Kp = kp;
        Ki = ki * TimeInSec;
        Kd = kd / TimeInSec;
    }

    /*
     * Set the new sample time, which represents the actual time between two calculations
     *
     * @param NewSampleTime in microseconds
     */
    void SetSampleTime(int NewSampleTime)
    {
        if (NewSampleTime > 0)
        {
            double ratio = (double)NewSampleTime / (double)sampleTime;
            Ki *= ratio;
            Kd /= ratio;
            sampleTime = (unsigned long)NewSampleTime;
        }
    }

    /*
     * Set new minimum and maximum output limits
     *
     * @param MinOut new minimum output limit, default is 0
     * @param MaxOut new minimum output limit, default is 255
     */
    void setOutputLimits(float MinOut, float MaxOut)
    {
        if (MinOut > MaxOut)
            return;
        minOutput = MinOut;
        maxOutput = MaxOut;

        if (piTerm > maxOutput)
            piTerm = maxOutput;
        else if (piTerm < minOutput)
            piTerm = minOutput;

        if (Output > maxOutput)
            Output = maxOutput;
        else if (Output < minOutput)
            Output = minOutput;
    }

    /*
     * Set new operating mode
     *
     * @param newOPMode either AUTOMATIC for PID control, or MANUAL to disable PID control
     */
    void setOperatingMode(bool newOPMode)
    {
        if (opMode == MANUAL && newOPMode == AUTOMATIC)
        {
            initialize();
        }
        opMode = newOPMode;
    }

    /*
     * Set new calculating mode
     *
     * @param newMode either PONE for "Proportional on Error" calculating, or PONM for "Proportional on Measurement" calculating
     */
    void setCalculationMode(bool newMode)
    {
        pMode = newMode;
        initialize();
    }

    /*
     * Calculates output in a set interval (default 10us), has to be called every loop()-routine
     *
     * @return calculated PID output
     */
    void calculate()
    {
        if (opMode == MANUAL)
            return;

        unsigned long now = micros();
        elapsedTime = (now - lastTime);

        if (elapsedTime >= sampleTime)
        {
            Input = *input;
            SetPoint = *setPoint;

            error = SetPoint - Input;
            dInput = (Input - lastInput);
            piTerm += (Ki * error);

            if (pMode == PONM)
                piTerm -= (Kp * dInput);

            if (piTerm > maxOutput)
                piTerm = maxOutput;
            else if (piTerm < minOutput)
                piTerm = minOutput;

            if (pMode == PONE)
                Output = (Kp * error);
            else
                Output = 0;

            Output += piTerm - Kd * dInput;

            if (Output > maxOutput)
                Output = maxOutput;
            else if (Output < minOutput)
                Output = minOutput;

            *output = Output;

            lastInput = Input;
            lastTime = now;
        }
    }
};