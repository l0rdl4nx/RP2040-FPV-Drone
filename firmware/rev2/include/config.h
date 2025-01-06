struct config
{
public:
#define PITCH 0
#define ROLL 1
#define YAW 2
#define THR 3

#define KP 0
#define KI 1
#define KD 2
#define MINOUT 3
#define MAXOUT 4

#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3

#define OFF 0
#define ON 1

#define X 0
#define Y 1
#define Z 2

    /*
     * Calbration values from/for the MPU calibration
     *
     * @param 0 X
     * @param 1 Y
     * @param 2 Z
     */
    float acc[3],
          gyro[3],
          mag[3],
          magScale[3];

    /*
     * Parameters for PID controllers
     *
     * @param 0 Kp (progressive), default 1
     * @param 1 Ki (integral), default 1
     * @param 2 Kd (deherative), default 1
     * @param 3 Minimal limit for PID output, default -200
     * @param 4 Maximum limit for PID output, default 200
     */
    float throttle[5] = {1, 1, 1, -200, 200},
          pitch[5] = {1, 1, 1, -200, 200},
          roll[5] = {1, 1, 1, -200, 200},
          yaw[5] = {1, 1, 1, -200, 200};
};