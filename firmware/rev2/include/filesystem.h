#include <ArduinoJson.h>
#include <LittleFS.h>
#include <config.h>

class filesystem
{
private:
    const char *cal_path = "/data/calibration.json";
    const char *pid_path = "/data/pid.json";

    int cal_params_count = 3;
    int pid_params_count = 5;

    boolean debug = false;

public:
#define SUCCEEDED 0
#define FAILED_JSON 1
#define FAILED_FILE_OPEN 2

    filesystem()
    {
    }

    /*
     * Mounts the filesystem for usage. No formating on fail is done.
     *
     * @return true if mounting was successful, false if it failed
     */
    boolean mount()
    {
        LittleFSConfig cfg;
        cfg.setAutoFormat(false);
        LittleFS.setConfig(cfg);

        return LittleFS.begin();
    }

    int getCalibration(config *conf)
    {
        File f = LittleFS.open(cal_path, "r");
        JsonDocument doc;

        DeserializationError err = deserializeJson(doc, f);

        if (!f)
            return FAILED_FILE_OPEN;
        if (err)
            return FAILED_JSON;

        for (int i = 0; i < cal_params_count; i++)
            conf->acc[i] = doc["acc"][i];
        for (int i = 0; i < cal_params_count; i++)
            conf->gyro[i] = doc["gyro"][i];
        for (int i = 0; i < cal_params_count; i++)
            conf->mag[i] = doc["mag"][i];
        for (int i = 0; i < cal_params_count; i++)
            conf->magScale[i] = doc["magScale"][i];

        f.close();

        return SUCCEEDED;
    }

    int saveCalibration(config *conf)
    {
        File f = LittleFS.open(cal_path, "w");
        JsonDocument doc;

        for (int i = 0; i < cal_params_count; i++)
            doc["acc"][i] = conf->acc[i];
        for (int i = 0; i < cal_params_count; i++)
            doc["gyro"][i] = conf->gyro[i];
        for (int i = 0; i < cal_params_count; i++)
            doc["mag"][i] = conf->mag[i];
        for (int i = 0; i < cal_params_count; i++)
            doc["magScale"][i] = conf->magScale[i];

        int serializeErr = serializeJson(doc, f);

        if (!f)
            return FAILED_FILE_OPEN;
        if (serializeErr == 0)
            return FAILED_JSON;

        f.close();

        return SUCCEEDED;
    }

    int getPID(config *conf)
    {
        File f = LittleFS.open(pid_path, "r");
        JsonDocument doc;

        DeserializationError err = deserializeJson(doc, f);

        if (!f)
            return FAILED_FILE_OPEN;
        if (err)
            return FAILED_JSON;

        for (int i = 0; i < pid_params_count; i++)
            conf->throttle[i] = doc["throttle"][i];
        for (int i = 0; i < pid_params_count; i++)
            conf->pitch[i] = doc["pitch"][i];
        for (int i = 0; i < pid_params_count; i++)
            conf->roll[i] = doc["roll"][i];
        for (int i = 0; i < pid_params_count; i++)
            conf->yaw[i] = doc["yaw"][i];

        f.close();

        return SUCCEEDED;
    }

    int savePID(config *conf)
    {
        File f = LittleFS.open(pid_path, "w");
        JsonDocument doc;

        for (int i = 0; i < pid_params_count; i++)
            doc["throttle"][i] = conf->throttle[i];
        for (int i = 0; i < pid_params_count; i++)
            doc["pitch"][i] = conf->pitch[i];
        for (int i = 0; i < pid_params_count; i++)
            doc["roll"][i] = conf->roll[i];
        for (int i = 0; i < pid_params_count; i++)
            doc["yaw"][i] = conf->yaw[i];

        int serializeErr = serializeJson(doc, f);

        if (!f)
            return FAILED_FILE_OPEN;
        if (serializeErr == 0)
            return FAILED_JSON;

        f.close();

        return SUCCEEDED;
    }

    void format()
    {
        boolean mountState = mount();
        if (!mountState) Serial.println("Mounting the filesystem failed.");
        else Serial.println("Mounting of the filesystem succedded.");

        boolean state = LittleFS.format();
        if (debug && state)
            Serial.println("Formatting successful.");
        else if (debug)
            Serial.println("Formatting failed.");
    }

    void setDebugForFS(boolean dbg)
    {
        debug = dbg;
    }
};