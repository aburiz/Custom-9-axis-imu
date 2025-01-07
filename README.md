# README: Building My 9-Axis IMU with MPU6050 and HMC5883L on ESP32

This is a step-by-step breakdown of how I built a 9-axis IMU using the MPU6050 and HMC5883L sensors connected to an ESP32. This project involved hardware assembly, coding, troubleshooting, and calibration. Here’s exactly what I did:

---

## **1. Components I Used**
- ESP32 microcontroller
- MPU6050 (accelerometer + gyroscope)
- HMC5883L (magnetometer)
- Breadboard and jumper wires
- USB cable for powering and programming the ESP32

---

## **2. Connecting the Hardware**

### **Step 1: Wire the MPU6050 to the ESP32**
- Connected the VCC pin of the MPU6050 to the ESP32’s 3.3V pin.
- Connected GND to GND.
- Connected SDA on the MPU6050 to GPIO 21 on the ESP32.
- Connected SCL on the MPU6050 to GPIO 22 on the ESP32.

### **Step 2: Wire the HMC5883L to the AUX Ports of the MPU6050**
- Connected the VCC and GND of the HMC5883L to 3.3V and GND (shared with the MPU6050).
- Connected SDA on the HMC5883L to AUX_DA on the MPU6050.
- Connected SCL on the HMC5883L to AUX_CL on the MPU6050.

### **Step 3: Double-Check Connections**
I checked that all wires were properly seated and that there were no loose connections. Sharing the I2C bus through the MPU6050’s passthrough mode was something I had to ensure worked later in code.

---

## **3. Setting Up the Code**

### **Step 1: Installed Required Libraries**
In the Arduino IDE, I installed the following libraries via **Sketch > Include Library > Manage Libraries**:
- `MPU6050 by Electronic Cats`
- `HMC5883L by Jarzebski`

### **Step 2: Wrote the Initialization Code**
I started by writing the code to initialize both sensors. The ESP32 communicates with the MPU6050, which handles the magnetometer passthrough. Below is the core of what I used:

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
const int HMC5883L_ADDRESS = 0x1E;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 initialized successfully");

    // Enable I2C bypass mode for HMC5883L
    mpu.setI2CBypassEnabled(true);

    // Initialize HMC5883L
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x00); // Configuration Register A
    Wire.write(0x70); // 8-average, 15 Hz output rate
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x01); // Configuration Register B
    Wire.write(0xA0); // Gain = 5
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x02); // Mode Register
    Wire.write(0x00); // Continuous measurement mode
    Wire.endTransmission();
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(HMC5883L_ADDRESS, 6);

    if (Wire.available() == 6) {
        int16_t mx = (Wire.read() << 8) | Wire.read();
        int16_t my = (Wire.read() << 8) | Wire.read();
        int16_t mz = (Wire.read() << 8) | Wire.read();

        Serial.print("Accel: X="); Serial.print(ax);
        Serial.print(" Y="); Serial.print(ay);
        Serial.print(" Z="); Serial.print(az);
        Serial.print(" | Gyro: X="); Serial.print(gx);
        Serial.print(" Y="); Serial.print(gy);
        Serial.print(" Z="); Serial.print(gz);
        Serial.print(" | Mag: X="); Serial.print(mx);
        Serial.print(" Y="); Serial.print(my);
        Serial.print(" Z="); Serial.println(mz);
    }

    delay(500);
}
```

### **Step 3: Uploaded and Tested**
- Uploaded the code to the ESP32.
- Opened the Serial Monitor (baud rate 115200) to verify the sensor outputs.

---

## **4. Results and Observations**

### **Accelerometer and Gyroscope Output**
- The accelerometer produced reasonable values, with the Z-axis showing ~16800 in a stationary position, indicating gravity.
- The gyroscope values hovered near zero when the device was stationary, as expected.

### **Magnetometer Output**
- The magnetometer readings were stable but needed calibration. They showed magnetic field strengths around ±140.

### **Calibration Issues**
- I noticed slight offsets in accelerometer and gyroscope readings, so I performed a manual calibration using offsets.
- The magnetometer required a figure-8 motion calibration to correct for hard and soft iron distortions.

---

## **5. Lessons Learned**
1. **Passthrough Mode Works Well:** The MPU6050’s AUX ports make it straightforward to integrate the HMC5883L without adding extra I2C wiring.
2. **Calibration is Critical:** Raw sensor data is often noisy or biased, so calibration steps (especially for the magnetometer) are essential for meaningful results.
3. **Power Stability:** I had some issues with inconsistent readings until I ensured stable power delivery to the sensors.

---

## **6. Next Steps**
- Implement a complementary or Kalman filter to combine accelerometer, gyroscope, and magnetometer data for accurate orientation and heading.
- Use the processed data to drive applications like a compass or a motion tracker.

---

This project was a great learning experience in combining sensors and handling I2C communication on an ESP32!

