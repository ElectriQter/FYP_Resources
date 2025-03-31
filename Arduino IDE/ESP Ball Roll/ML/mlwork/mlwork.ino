/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <ESP_Accelerometer_inferencing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// Settings
#define BTN_PIN             26         // Button pin
#define LEDP                 2        // Red LED pin

// Constants
#define SAMPLING_FREQ_HZ    100       // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
#define NUM_SAMPLES         100       // 100 samples at 100 Hz is 1 sec window

Adafruit_MPU6050 mpu;

static const float features[] = {
    // copy raw features here (for example from the 'Live classification' page)
    // see https://docs.edgeimpulse.com/docs/running-your-impulse-arduino
};


float f_in [] = {
    //paste raw data here
    1.6400, -8.8100, 3.9300, 0.1800, -0.2000, 0.1900, 1.6000, -8.8200, 3.9200, 0.2000, -0.1800, 0.1900, 1.5800, -8.8200, 3.8900, 0.2100, -0.1800, 0.1800, 1.5800, -8.8000, 3.9100, 0.2200, -0.1800, 0.1800, 1.5800, -8.8100, 4.0000, 0.2000, -0.1700, 0.1800, 1.5600, -8.8200, 4.0900, 0.1700, -0.1500, 0.1800, 1.5500, -8.8400, 4.1100, 0.1200, -0.1200, 0.1700, 1.5300, -8.8800, 4.0700, 0.0800, -0.1000, 0.1700, 1.4500, -8.9400, 4.0600, 0.0600, -0.0700, 0.1600, 1.4000, -8.8600, 3.9900, 0.0200, -0.0900, 0.1500, 1.4200, -8.8300, 3.9300, -0.0200, -0.0900, 0.1400, 1.4100, -8.7900, 3.8400, -0.0500, -0.1000, 0.1200, 1.4400, -8.7500, 3.7400, -0.0700, -0.1000, 0.1100, 1.4200, -8.7300, 3.7200, -0.0900, -0.1100, 0.1000, 1.4100, -8.7500, 3.6600, -0.1000, -0.1000, 0.0800, 1.4300, -8.8000, 3.6100, -0.1100, -0.1000, 0.0600, 1.4700, -8.8700, 3.5800, -0.1100, -0.1000, 0.0500, 1.5000, -8.9300, 3.5200, -0.1200, -0.0900, 0.0400, 1.5300, -8.9600, 3.4700, -0.1200, -0.0800, 0.0400, 1.5500, -8.9700, 3.4200, -0.1300, -0.0700, 0.0400, 1.5600, -8.9800, 3.3900, -0.1400, -0.0700, 0.0400, 1.5700, -8.9900, 3.3500, -0.1400, -0.0600, 0.0400, 1.5700, -8.9800, 3.2900, -0.1400, -0.0600, 0.0300, 1.5600, -9.0000, 3.2100, -0.1400, -0.0700, 0.0300, 1.5700, -9.0100, 3.1900, -0.1200, -0.0800, 0.0200, 1.5600, -9.0000, 3.2500, -0.1100, -0.0900, 0.0200, 1.5400, -8.9800, 3.3100, -0.1100, -0.0900, 0.0200, 1.5400, -8.9600, 3.3100, -0.1300, -0.0800, 0.0200, 1.5400, -8.9700, 3.2900, -0.1500, -0.0800, 0.0200, 1.5400, -9.0200, 3.2700, -0.1600, -0.0800, 0.0200, 1.5600, -9.0800, 3.2700, -0.1700, -0.0700, 0.0200, 1.5800, -9.1300, 3.2600, -0.1700, -0.0600, 0.0200, 1.6000, -9.1800, 3.2400, -0.1600, -0.0400, 0.0300, 1.5800, -9.1600, 3.2400, -0.1400, -0.0300, 0.0400, 1.5700, -9.1000, 3.2100, -0.1300, -0.0300, 0.0400, 1.5600, -9.0800, 3.1800, -0.1200, -0.0300, 0.0400, 1.5400, -9.0900, 3.2000, -0.1000, -0.0400, 0.0400, 1.5500, -9.0900, 3.2200, -0.0900, -0.0400, 0.0400, 1.5500, -9.0700, 3.2200, -0.0800, -0.0400, 0.0400, 1.5300, -9.0500, 3.2200, -0.0800, -0.0400, 0.0500, 1.5300, -9.0500, 3.2400, -0.0800, -0.0400, 0.0500, 1.5200, -9.0300, 3.2400, -0.0700, -0.0300, 0.0500, 1.5000, -9.0100, 3.2200, -0.0700, -0.0300, 0.0400, 1.4900, -9.0100, 3.1900, -0.0700, -0.0300, 0.0400, 1.5000, -9.0100, 3.1700, -0.0700, -0.0300, 0.0300, 1.5000, -9.0300, 3.1800, -0.0800, -0.0300, 0.0300, 1.5100, -9.0300, 3.1900, -0.0800, -0.0300, 0.0400, 1.5000, -9.0200, 3.1800, -0.0900, -0.0300, 0.0400, 1.5300, -9.0400, 3.1700, -0.0800, -0.0300, 0.0300, 1.5400, -9.0900, 3.1600, -0.0800, -0.0300, 0.0200, 1.5600, -9.1400, 3.1400, -0.0700, -0.0300, 0.0200, 1.5800, -9.1900, 3.1400, -0.0600, -0.0300, 0.0200, 1.6100, -9.2200, 3.1400, -0.0500, -0.0300, 0.0200, 1.6200, -9.2200, 3.1300, -0.0400, -0.0300, 0.0300, 1.6100, -9.2000, 3.1200, -0.0400, -0.0300, 0.0300, 1.5900, -9.1700, 3.1100, -0.0500, -0.0300, 0.0400, 1.5800, -9.1400, 3.1100, -0.0600, -0.0300, 0.0400, 1.5500, -9.1200, 3.1000, -0.0800, -0.0200, 0.0400, 1.5400, -9.0900, 3.0900, -0.0900, -0.0200, 0.0400, 1.5300, -9.0800, 3.0800, -0.0900, -0.0200, 0.0400, 1.5300, -9.0800, 3.0900, -0.0900, -0.0300, 0.0400, 1.5400, -9.0900, 3.1000, -0.0900, -0.0200, 0.0400, 1.5400, -9.1100, 3.1100, -0.0800, -0.0200, 0.0300, 1.5300, -9.0900, 3.1100, -0.0700, -0.0200, 0.0300, 1.5100, -9.0500, 3.1100, -0.0700, -0.0300, 0.0400, 1.4900, -9.0300, 3.1300, -0.0600, -0.0300, 0.0300, 1.4900, -9.0300, 3.1600, -0.0700, -0.0300, 0.0300, 1.4800, -9.0300, 3.1600, -0.0700, -0.0300, 0.0300, 1.4700, -9.0300, 3.1500, -0.0700, -0.0300, 0.0300, 1.4600, -8.9900, 3.1500, -0.0700, -0.0300, 0.0300, 1.4500, -8.9600, 3.1600, -0.0700, -0.0200, 0.0300, 1.4400, -8.9600, 3.1900, -0.0800, -0.0200, 0.0200, 1.4600, -8.9700, 3.2000, -0.0800, -0.0100, 0.0200, 1.4600, -8.9800, 3.1800, -0.0900, 0.0000, 0.0200, 1.4600, -8.9800, 3.1600, -0.0800, 0.0000, 0.0200, 1.4500, -8.9600, 3.1600, -0.0800, 0.0100, 0.0100, 1.4500, -8.9400, 3.1500, -0.0700, 0.0100, 0.0100, 1.4500, -8.9300, 3.1300, -0.0600, 0.0100, 0.0000, 1.4400, -8.9400, 3.1300, -0.0500, 0.0000, 0.0000, 1.4300, -8.9300, 3.1100, -0.0500, 0.0000, 0.0000, 1.4300, -8.9000, 3.1200, -0.0400, -0.0100, 0.0000, 1.4200, -8.8500, 3.1300, -0.0500, -0.0100, -0.0100, 1.4100, -8.8100, 3.1300, -0.0600, -0.0200, -0.0100, 1.4100, -8.8000, 3.1500, -0.0800, -0.0300, -0.0200, 1.4200, -8.8200, 3.1900, -0.1000, -0.0300, -0.0300, 1.4400, -8.8800, 3.1900, -0.1200, -0.0200, -0.0400, 1.4700, -8.9600, 3.2000, -0.1400, -0.0100, -0.0400, 1.5000, -9.0400, 3.1800, -0.1600, 0.0100, -0.0400, 1.5200, -9.1100, 3.1500, -0.1600, 0.0200, -0.0400, 1.5200, -9.1500, 3.1200, -0.1600, 0.0400, -0.0400, 1.5400, -9.1700, 3.0800, -0.1500, 0.0400, -0.0400, 1.5600, -9.2000, 3.0600, -0.1400, 0.0400, -0.0400, 1.5900, -9.2500, 3.0400, -0.1200, 0.0400, -0.0400, 1.6100, -9.3000, 3.0500, -0.1000, 0.0400, -0.0400, 1.6200, -9.3300, 3.0500, -0.0800, 0.0400, -0.0300, 1.6100, -9.3300, 3.0600, -0.0700, 0.0400, -0.0300, 1.6000, -9.3000, 3.0400, -0.0600, 0.0400, -0.0200, 1.5900, -9.2700, 3.0200, -0.0500, 0.0400, -0.0100, 1.5800, -9.2400, 3.0300, -0.0400, 0.0400, 0.0000, 1.5800, -9.2100, 3.0500, -0.0400, 0.0400, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000

  };

/**
 * @brief      Copy raw feature data in out_ptr
 *             Function called by inference library
 *
 * @param[in]  offset   The offset
 * @param[in]  length   The length
 * @param      out_ptr  The out pointer
 *
 * @return     0
 */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  
  //float [] sensordata;

  // retrieve from accelerometer and dump into sensordata  
    memcpy(out_ptr,f_in + offset, length * sizeof(float));
    return 0;
}

void update_f_in(float new_data[], size_t new_data_length) {
    // Ensure we do not exceed the size of f_in
    size_t max_length = sizeof(f_in) / sizeof(f_in[0]);
    size_t length_to_copy = (new_data_length < max_length) ? new_data_length : max_length;

    // Copy new data into f_in
    memcpy(f_in, new_data, length_to_copy * sizeof(float));
}

void print_inference_result(ei_impulse_result_t result);

/**
 * @brief      Arduino setup function
 */
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    //Enable Button Pin
    pinMode(BTN_PIN,INPUT_PULLDOWN);

    //Enable LED PIN 
    pinMode(LEDP, OUTPUT);
    digitalWrite (LEDP, LOW);

    //MPU6050 setup
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

/**
 * @brief      Arduino main function
 */
void loop()
{
float new_sensor_data[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]={};

float ax, ay,az;    //variable for accelerometer
  float rx,ry,rz;     //variable for gyro 

  unsigned long timestamp;
  unsigned long start_timestamp;

  while (digitalRead(BTN_PIN) == 1)
  {
    //turn on led
    digitalWrite(LEDP,HIGH);

    //Print Header
    //Serial.println("timestamp,accX,accY,axxZ,gyrX,gyrY,gyrZ");

    //record samples in buffer
    start_timestamp = millis();
  for (int i=0; i<NUM_SAMPLES; i++)
    {
      //take timestamp 
      timestamp=millis();

      //read accelerometer (m/s^2)
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      ax= a.acceleration.x;
      ay= a.acceleration.y;
      az= a.acceleration.z;

      //gyroscope data (in degrees/sec)
      rx= g.gyro.x;
      ry= g.gyro.y;
      rz= g.gyro.z;    

      new_sensor_data[i * 6 + 0] = ax;
      new_sensor_data[i * 6 + 1] = ay;
      new_sensor_data[i * 6 + 2] = az;
      new_sensor_data[i * 6 + 3] = rx;
      new_sensor_data[i * 6 + 4] = ry;
      new_sensor_data[i * 6 + 5] = rz;

/*
      //print csv data with timestamp
      Serial.print(timestamp - start_timestamp);
      Serial.print(",");
      Serial.print(ax);
      Serial.print(",");
      Serial.print(ay);
      Serial.print(",");
      Serial.print(az);
      Serial.print(",");
      Serial.print(rx);
      Serial.print(",");
      Serial.print(ry);
      Serial.print(",");
      Serial.println(rz);  */

      // Wait just long enough for our sampling period
      while (millis() < timestamp + SAMPLING_PERIOD_MS);
    }



    // Update f_in with new sensor data
    update_f_in(new_sensor_data, sizeof(new_sensor_data) / sizeof(new_sensor_data[0]));

    // Continue with inference

    ei_printf("Edge Impulse standalone inferencing (Arduino)\n");

    if (sizeof(f_in) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
        delay(1000);
        return;
    }

    ei_impulse_result_t result = { 0 };

    // the features are stored into flash, and we don't want to load everything into RAM
    signal_t f_in_signal;
    f_in_signal.total_length = sizeof(f_in) / sizeof(f_in[0]);
    f_in_signal.get_data = &raw_feature_get_data;

    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&f_in_signal, &result, false /* debug */);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }

    // print inference return code
    ei_printf("run_classifier returned: %d\r\n", res);
    print_inference_result(result);

    delay(1000);

    //turn off LED
    digitalWrite(LEDP,LOW);
  }
}

void print_inference_result(ei_impulse_result_t result) {

    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    // Print the prediction results (object detection)
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif

}