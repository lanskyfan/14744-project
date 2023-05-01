#include "myApp.h"
#include "interface.h"
// sensor libraries for data interperting
#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"
#include "meas_magn/resources.h"
#include "meas_imu/resources.h"
#include <ui_ind/resources.h>

// This code is modified by Yifan Lan (Andrew ID: yifanlan)

//Commands formated as byte array with [Command, data?...] (data optional)
enum Commands 
{
    HELLO = 0,
    BEGIN_SUB=1,
    END_SUB=2,
    BLINK=3,
};
//Responses formated as byte array with [Response, tag, data?...] (data optional)
enum Responses 
{
    COMMAND_RESULT = 1,
    DATA = 2,
    ERROR = 3,
};
          
const char IMUPath[]="/Meas/IMU6/52"; // path to IMU data, using 52 frequency
const char GYROPath[]="/Meas/Gyro/52"; // path to gyroscope data, using 52 frequency
const uint8_t DEFAULT_REFERENCE=99; //appears as 63 in hex
const uint8_t IMU_REF=20;
const int8_t BEGIN_THRESHOLD=6; // acceleration threshold for split step beginning
const int8_t END_THRESHOLD=10; // acceleration threshold for split step ending
const int8_t SPLIT_STEP_THRESHOLD=80; // the threshold to differentiate between split steps and other footworks
const uint32 GOOD_STEP_LENGTH=200; // the maximum time (ms) for performing a good split step
const uint8_t IMU_TAG = 5; // tag for gyroscope data
const bool TEST = false; // whether enable test mode


bool begin = false; // whether the split step process has begun
uint32 beginTime; // the starting timestamp of split step
float maxZGyro = 0; // the maximum gyroscope magnitude detected during split step
float beginZAcc; // beginning z-axis acc
float endZAcc; // ending z-axis acc

void myApp::handleCommand(uint8_t cmd, const uint8_t values[], size_t len){
    switch (cmd)
    {
        case Commands::HELLO:
        {
            // Hello response, for use as a sanity check <3
            uint8_t helloMsg[] = {'H','e','l','l','o'};
            //tags aren't explicitly necessary, but they're a good way of grouping responses.
            //The included console uses them to filter and format responses.
            uint8_t tag=1;
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
        }
        break;
        // This case handles the split step analysis
        case Commands::BEGIN_SUB:
        {
            //unsubscribes to prevent duplicate subscriptions
            unsubscribe(DEFAULT_REFERENCE);
            uint8_t msg[] = "subscribe";
            sendPacket(msg, sizeof(msg), IMU_TAG, Responses::COMMAND_RESULT);
            //subscribes to the IMU data

            subscribe(IMUPath, sizeof(IMUPath), IMU_REF);
        }
        break;
        case Commands::END_SUB:
        {
            //unsubscribes
            uint8_t msg[] = "unsubscribe";
            sendPacket(msg, sizeof(msg), IMU_TAG, Responses::COMMAND_RESULT);
            unsubscribe(IMU_REF);
        }
        break;
        case Commands::BLINK:
        {
            //  blinking the Movesense device's LED light 
            // 3 times with 2 second pauses in between in each blink.
            ledSetPattern(1000, 2000, 3);
        }
        break;
    }
}

void myApp::processData(wb::ResourceId resourceId, const wb::Value &value){
    // only process data from IMU subscription
    if(findDataSub(resourceId)->clientReference != IMU_REF)
        return;
    const WB_RES::IMU6Data &data = value.convertTo<WB_RES::IMU6Data&>();
    const wb::Array<wb::FloatVector3D> &accData = data.arrayAcc;
    const wb::Array<wb::FloatVector3D> &gyroData = data.arrayGyro;
    // average gyroscope data
    float averageZGyro=0;
    for (size_t i=0; i<gyroData.size();i++) {
        wb::FloatVector3D a = gyroData[i];
        // float gyro = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
        // to only get the magnitude and ignore the direction
        averageZGyro += sqrt(a.z*a.z);
    }
    averageZGyro /= gyroData.size();

    // average acceleration data
    float averageAccMagnitude=0;
    float averageZAcc=0;
    for (size_t i=0; i<accData.size();i++) {
        wb::FloatVector3D a = accData[i];
        float acc = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
        averageZAcc += a.z;
        averageAccMagnitude += acc;
    }
    averageAccMagnitude /= accData.size();
    averageZAcc /= accData.size();

    // this marks the beginning of the zero-acceleration period
    if (averageAccMagnitude < BEGIN_THRESHOLD) {
        if (!begin) {
            // reset all the value. Record the initial timestamp to calculate the height of jump
            begin = true;
            maxZGyro = 0;
            beginZAcc = averageZAcc;
            beginTime = data.timestamp;
            // for debugging
            if (TEST) {
                char newMsg[] = "zero-g begins, acc is ";
                sendWithNumber(newMsg, (int) averageAccMagnitude);
            }
        } else {
            endZAcc = averageZAcc;
        }
        // calculate the maximum of gyroscope data during the zero-acceleration period
        maxZGyro = fmax(maxZGyro, averageZGyro);
        return;
    }

    // this markes the end of the zero-acceleration period
    if (begin && averageAccMagnitude > END_THRESHOLD) {
        begin = false;

        // if the gyroscope data falls in the range of split step
        if (maxZGyro < SPLIT_STEP_THRESHOLD) {
            // calculate the total duration to get the height of jump
            uint32 endTime = data.timestamp;
            uint32 duration = endTime - beginTime;

            // good split step.
            if (duration <= GOOD_STEP_LENGTH) {
                uint8_t splitMsg[] = "You perform an excellent split step";
                sendPacket(splitMsg, sizeof(splitMsg), IMU_TAG, Responses::COMMAND_RESULT);
            // bad split step only consider the case where the user jump too high due to the limitation of the sensor
            } else {
                uint8_t splitMsg[] = "You jumped too high for your split step";
                sendPacket(splitMsg, sizeof(splitMsg), IMU_TAG, Responses::COMMAND_RESULT);
            }
            // debug purpose
            if (TEST) {
                char splitMsg[] = "split step detected. maxZGyro = ";
                sendWithNumber(splitMsg, (int) maxZGyro);
            }

        } else {
            // debug purpose
            if (TEST) {
                char splitMsg[] = "this is not split, maxZGyro = ";
                sendWithNumber(splitMsg, (int) maxZGyro);
            }
        }
    }
}

// the purpose of the function is to send an int to the console
// this will help me to debug and fine tune the parameters
void myApp::sendWithNumber(char *message, int number){
    int nDigits = floor(log10(number)) + 1;
    char numStr[12];
    sprintf(numStr, "%d", number);
    int len = strlen(message);
    uint8_t output[50];
    for (int i = 0; i < len; i++) {
        output[i] = uint8_t(message[i]);
    }
    for (int i = len; i < 12 + len; i++) {
        output[i] = uint8_t(numStr[i - len]);
    }
    sendPacket(output, len + nDigits, IMU_TAG, Responses::COMMAND_RESULT);

}

