/* mbed Microcontroller Library
 * Copyright (c) 2017-2019 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mbed.h"
#include "platform/Callback.h"
#include "events/EventQueue.h"
#include "ble/BLE.h"
#include "gatt_server_process.h"
#include "mbed-trace/mbed_trace.h"
#include <iostream>
#include "rtos.h"


using mbed::callback;
using namespace std::literals::chrono_literals;
// DigitalOut ledRed(A2);
Thread thread_1;
Thread thread_2(osPriorityNormal1); // 24 + 1

/////////////////  buzzer  /////////////////
// PIN PWM ARD11 exclusive to PWM
// For more configuration see: https://os.mbed.com/docs/mbed-os/v6.16/apis/pwmout.html
PwmOut myBuzzer(D11);
DigitalOut myBuzzerLed(A2);
bool activeMyBuzzer = 1;
// Frecuency Do 3846 us -> 260 Hz -> Low
// Frecuency Do 2857 us -> 350 Hz -> Medium
// Frecuency Si 2028 us -> 493 Hz -> Heigh
int periodMyBuzzer = 2857;
int freqMyBuzzer = 350;
// Low level 0.05f 5% duty cycle
// High level 0.95f 95% duty cycle
float levelMyBuzzer = 0.05f;

/////////////////  ultrasonic sensor HC-SR04 /////////////////
// Min range 2 cm
Timer sonar;
DigitalOut triggerUltrasonicSensor(A1);
DigitalIn  echoUltrasonicSensor(A0);
int correctionUltrasonicSensor = 0;
int distanceUltrasonicSensor = 0;
int distanceUltrasonicSensorBackup = 0;
int freqUpdateUltrasonicSensor = 1000;      // 1000 ms

/////////////////  threshold /////////////////
// medium risk             15 cm
int mediumRiskDistance = 15;
// Risk high                5 cm
int highRiskDistance = 5;

/**
 * A MyGuideService service that demonstrate how to
 * set parameter in board.
 */
class MyGuideService : public ble::GattServer::EventHandler {
public:
    MyGuideService() :
        switchOnLed("485f4145-52b9-4644-af1f-7a6b9322490f", 0),

        _my_guide_gatt_service(
            /* uuid */ "51311102-030e-485f-b122-f8f381aa84ed",
            /* characteristics */ characteristics,
            /* numCharacteristics */ sizeof(characteristics) /
                                     sizeof(characteristics[0])
        )
    {
        /* update internal pointers (value, descriptors and characteristics array) */
        characteristics[0] = &switchOnLed;

        /* setup authorization handlers */
        switchOnLed.setWriteAuthorizationCallback(this, &MyGuideService::authorize_client_write);
    }

    void start(BLE &ble, events::EventQueue &event_queue)
    {
        _server = &ble.gattServer();
        _event_queue = &event_queue;

        printf("Registering service My Guide ...\r\n");
        ble_error_t err = _server->addService(_my_guide_gatt_service);

        if (err) {
            printf("Error %u during demo service registration.\r\n", err);
            return;
        }

        /* register handlers */
        _server->setEventHandler(this);

        printf("My Guide service registered\r\n");
        printf("service handle: %u\r\n", _my_guide_gatt_service.getHandle());
        printf("switchOnLed characteristic value handle id %u\r\n", switchOnLed.getValueHandle());

        // _event_queue->call_every(1000ms, callback(this, &MyGuideService::increment_second));
    }

    /* GattServer::EventHandler */
private:
    /**
     * Handler called when a notification or an indication has been sent.
     */
    void onDataSent(const GattDataSentCallbackParams &params) override
    {
        printf("sent updates\r\n");
    }

    /**
     * Handler called after an attribute has been written.
     */
    void onDataWritten(const GattWriteCallbackParams &params) override
    {
        printf("data written:\r\n");
        printf("connection handle: %u\r\n", params.connHandle);
        printf("attribute handle: %u", params.handle);
        if (params.handle == switchOnLed.getValueHandle()) {
            printf(" (switchOnLed characteristic)\r\n");
        } else {
            printf("\r\n");
        }
        printf("write operation: %u\r\n", params.writeOp);
        printf("offset: %u\r\n", params.offset);
        printf("length: %u\r\n", params.len);
        printf("data: ");

        for (size_t i = 0; i < params.len; ++i) {
            printf("%d\r\n", params.data[i]);
        }

        printf("\r\n");
    }

    /**
     * Handler called after an attribute has been read.
     */
    void onDataRead(const GattReadCallbackParams &params) override
    {
        printf("data read:\r\n");
        printf("connection handle: %u\r\n", params.connHandle);
        printf("attribute handle: %u\r\n", params.handle);
        if (params.handle == switchOnLed.getValueHandle()) {
            printf("(switchOnLed characteristic)\r\n");
        } else {
            printf("\r\n");
        }
    }

    /**
     * Handler called after a client has subscribed to notification or indication.
     *
     * @param handle Handle of the characteristic value affected by the change.
     */
    void onUpdatesEnabled(const GattUpdatesEnabledCallbackParams &params) override
    {
        printf("update enabled on handle %d\r\n", params.attHandle);
    }

    /**
     * Handler called after a client has cancelled his subscription from
     * notification or indication.
     *
     * @param handle Handle of the characteristic value affected by the change.
     */
    void onUpdatesDisabled(const GattUpdatesDisabledCallbackParams &params) override
    {
        // printf("update disabled on handle %d\r\n", params.attHandle);
    }

    /**
     * Handler called when an indication confirmation has been received.
     *
     * @param handle Handle of the characteristic value that has emitted the
     * indication.
     */
    void onConfirmationReceived(const GattConfirmationReceivedCallbackParams &params) override
    {
        // printf("confirmation received on handle %d\r\n", params.attHandle);
    }

private:
    /**
     * Handler called when a write request is received.
     *
     * This handler verify that the value submitted by the client is valid before
     * authorizing the operation.
     */
    void authorize_client_write(GattWriteAuthCallbackParams *e)
    {
        uint8_t firstCharacter = e->data[0];
        uint8_t idCharacteristic = e->handle;
        printf("characteristic %u write authorization\r\n", e->handle);

        if (e->offset != 0) {
            printf("Error invalid offset\r\n");
            e->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_INVALID_OFFSET;
            return;
        }

        if (e->len != 1) {
            printf("Error invalid len\r\n");
            e->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_INVALID_ATT_VAL_LENGTH;
            return;
        }
        // printf("e->data[0]: %d\r\n", e->data[0]);
        // printf("e->data[1]: %d\r\n", e->data[1]);

        // First service
        // Handle data received 
        if(idCharacteristic == 13){
            if(firstCharacter == 0){
                activeMyBuzzer = 0;
                myBuzzerLed = 0;
                myBuzzer.suspend();
            }else if (firstCharacter == 1){
                activeMyBuzzer = 1;
                myBuzzerLed = 1;
                myBuzzer.resume();
            }
        }
        // Fin handle

        if ((e->data[0] >= 60) ||
            ((e->data[0] >= 24) && (e->handle == switchOnLed.getValueHandle()))) {
            printf("Error invalid data\r\n");
            e->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_WRITE_NOT_PERMITTED;
            return;
        }

        e->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
    }

private:
    /**
     * Read, Write, Notify, Indicate  Characteristic declaration helper.
     *
     * @tparam T type of data held by the characteristic.
     */
    template<typename T>
    class ReadWriteNotifyIndicateCharacteristic : public GattCharacteristic {
    public:
        /**
         * Construct a characteristic that can be read or written and emit
         * notification or indication.
         *
         * @param[in] uuid The UUID of the characteristic.
         * @param[in] initial_value Initial value contained by the characteristic.
         */
        ReadWriteNotifyIndicateCharacteristic(const UUID & uuid, const T& initial_value) :
            GattCharacteristic(
                /* UUID */ uuid,
                /* Initial value */ &_value,
                /* Value size */ sizeof(_value),
                /* Value capacity */ sizeof(_value),
                /* Properties */ GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
                                 GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
                                 GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY |
                                 GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE,
                /* Descriptors */ nullptr,
                /* Num descriptors */ 0,
                /* variable len */ false
            ),
            _value(initial_value) {
        }

        /**
         * Get the value of this characteristic.
         *
         * @param[in] server GattServer instance that contain the characteristic
         * value.
         * @param[in] dst Variable that will receive the characteristic value.
         *
         * @return BLE_ERROR_NONE in case of success or an appropriate error code.
         */
        ble_error_t get(GattServer &server, T& dst) const
        {
            uint16_t value_length = sizeof(dst);
            return server.read(getValueHandle(), &dst, &value_length);
        }

        /**
         * Assign a new value to this characteristic.
         *
         * @param[in] server GattServer instance that will receive the new value.
         * @param[in] value The new value to set.
         * @param[in] local_only Flag that determine if the change should be kept
         * locally or forwarded to subscribed clients.
         */
        ble_error_t set(GattServer &server, const uint8_t &value, bool local_only = false) const
        {
            return server.write(getValueHandle(), &value, sizeof(value), local_only);
        }

    private:
        uint8_t _value;
    };

private:
    GattServer *_server = nullptr;
    events::EventQueue *_event_queue = nullptr;

    GattService _my_guide_gatt_service;
    GattCharacteristic* characteristics[1];

    ReadWriteNotifyIndicateCharacteristic<uint8_t> switchOnLed;
};

void triggerUltrasonicSensorFunction()
{
    // Calibration
    sonar.reset();
    // measure actual software polling timer delays
    // delay used later in time correction
    // start timer
    sonar.start();
    // min software polling delay to read echo pin
    while (echoUltrasonicSensor==2) {};
    // stop timer
    sonar.stop();
    // read timer
    correctionUltrasonicSensor = sonar.elapsed_time().count();
    printf("Approximate software overhead timer delay is %d uS\n\r",correctionUltrasonicSensor);

    int rangeDistance = mediumRiskDistance - highRiskDistance;  // 10 cm e.g.
    float difference = 0.0f, auxLimitMaximum = 0.0f;
    float newDutyCycle = 0.0f, newFreq = 0.0f;
    int newPeriod = periodMyBuzzer;
    while(1)
    {
        // Generate pulse in trigger
        triggerUltrasonicSensor = 1;
        sonar.reset();
        wait_us(10);
        triggerUltrasonicSensor = 0;

        //wait for echo high
        while (echoUltrasonicSensor==0) {};
        sonar.start();
        //wait for echo low
        while (echoUltrasonicSensor==1) {};
        sonar.stop();
        distanceUltrasonicSensor = (sonar.elapsed_time().count() - correctionUltrasonicSensor)/58.0;  // return in us
        
        printf("%d cm\r\n", distanceUltrasonicSensor);

        // Active buzzer according to distance, modiying frequency and level
        // Handle risk
        if(highRiskDistance <= distanceUltrasonicSensor && distanceUltrasonicSensor <= mediumRiskDistance){
            // Active buzzer
            if(activeMyBuzzer){
                myBuzzer.resume();
                // 1º Calculate new duty cycle
                difference = mediumRiskDistance - distanceUltrasonicSensor;
                // printf("difference: %f \r\n", difference);
                auxLimitMaximum = rangeDistance - difference;
                // printf("auxLimitMaximum: %f \r\n", auxLimitMaximum);
                // Regla de tres
                newDutyCycle = (90 * auxLimitMaximum) / rangeDistance;
                // printf("regla de tres v2: %f \r\n", newDutyCycle);
                newDutyCycle = newDutyCycle + 5;
                newDutyCycle = 100 - newDutyCycle;
                newDutyCycle = newDutyCycle / 100;
                printf("newDutyCycle: %f \r\n", newDutyCycle);
                // printf("newDutyCycle: %.2f \r\n", newDutyCycle / 100);

                // 2º Calculate new frequency
                // Increase frequency when we get closer to limit, more closer => up frequency
                newFreq = (freqMyBuzzer * auxLimitMaximum) /rangeDistance;
                printf("newFreq %f\r\n", newFreq);
                newFreq = freqMyBuzzer + (freqMyBuzzer - newFreq);
                newPeriod = int ((1 / newFreq) * 1000000);
                printf("newPeriod %d\r\n", newPeriod);
                
                // Start with medium frequency
                if(distanceUltrasonicSensor != distanceUltrasonicSensorBackup){
                    myBuzzer.period_us(newPeriod); // Period in us
                    myBuzzer.write(newDutyCycle);
                }
                distanceUltrasonicSensorBackup = distanceUltrasonicSensor;
                // auto newDutyCycle = (distanceUltrasonicSensor * (95 - 5)) / (highRiskDistance - mediumRiskDistance);
                // myBuzzer.write(newDutyCycle);
            }
        // Without risk 
        }else if(distanceUltrasonicSensor > mediumRiskDistance){
            // Disable buzzer
            // activeMyBuzzer = 0;
            myBuzzer.suspend();
        }

        ThisThread::sleep_for(freqUpdateUltrasonicSensor);
    }
}

int main(void)
{
    printf("Start v78\r\n");    

    // Switch on led by default
    myBuzzerLed = 1;

    thread_1.start(triggerUltrasonicSensorFunction);
    
    mbed_trace_init();

    ////////////////// BLE ////////////////////
    BLE &ble = BLE::Instance();
    events::EventQueue event_queue;
    MyGuideService demo_service;

    /* this process will handle basic ble setup and advertising for us */
    GattServerProcess ble_process(event_queue, ble);

    /* once it's done it will let us continue with our demo */
    ble_process.on_init(callback(&demo_service, &MyGuideService::start));

    ble_process.start();

    return 0;
}

