/*
Student Name: Sebastian Cooper-Giannotta
Teacher Name: Mr. So
Title of Project: Motion & Distance Measurement & Scanning Device
Brief Description of Project: Utilizes accelerometers, an ultrasonic sensor and an e-paper display screen to track motion, measure distances between points, measure positions of points in an environment and display them in a 3d projection.
Date of completion: June 12, 2025
*/

#include <Vector_datatype.h>
#include <BasicLinearAlgebra.h>
#include <SPI.h>
#include <Ultrasonic.h>
#include <epd2in9.h>
#include <epdpaint.h>

int i; // Declare i integer for for loops
int j; // Declare j integer for nested for loops

const char powerControlAddress = 0x2D; // Declare power control address for SPI communication with the accelerometer
const char dataFormatAddress = 0x31; // Declare data formatting address for SPI communication with the accelerometer
const char accelDataFirstAddress = 0x32; // Declare address of the first component of acceleration data for SPI communication with the accelerometer

unsigned char inputBuffer[10]; // Declare buffer to hold values read from the ADXL345 registers

void writeRegister(int csPin, char registerAddress, char value){
  digitalWrite(csPin, LOW); // Set chip select pin low to signal the beginning of an SPI packet
 
  SPI.transfer(registerAddress); // Transfer the register address over SPI
  SPI.transfer(value); // Transfer the desired register value over SPI
 
  digitalWrite(csPin, HIGH); // Set the chip Select pin high to signal the end of an SPI packet
}

void readRegister(int csPin, char registerAddress, int byteCount){
  registerAddress = 0x80 | registerAddress; // Ensure most significant bit of the register address is set for read operation

  if (byteCount > 1) { // Check for multi-byte read
    registerAddress = registerAddress | 0x40; // Set bit 6
  }
 
  digitalWrite(csPin, LOW); // Set the chip select pin low to start an SPI packet
  SPI.transfer(registerAddress); // Transfer the starting register address that needs to be read
 
  for (j = 0; j < byteCount; j++) { // Loop through the specific amount of bytes
    inputBuffer[j] = SPI.transfer(0x00); // Read specific byte from register and store it into the input buffer
  }
 
  digitalWrite(csPin, HIGH); // Set the chip select pin high to end the SPI packet
}

const int accelerationDataByteCount = 6; // Declare amount of bits
const float accelerometerReadingScale = 256; // Declare the max reading scale for the accelerometer (0 = 0g, 256 = 1g)
const float gravityAccelMagnitude = 9.81; // Declare the magnitude of acceleration due to gravity
const vec3_t gravityAccelVector = {0, -gravityAccelMagnitude, 0}; // Declare gravity acceleration vector

// Declare accelerometer struct to encapsulate accelerometer properties and methods
struct Accelerometer {
  int csPin; // Declare chip select pin
  vec3_t offset; // Declare accelerometer offset from the origin
  int* axesInfo; // Declare axes info array to format the read acceleration with different axes/signs

  vec3_t rawAccel; // Declare the raw read acceleration of the accelerometer
  vec3_t formattedAccel; // Declare the formatted acceleration of the accelerometer
 
  int calibrationCount = 0; // Declare the amount of calibration updates for the accelerometer
  vec3_t calibratedAccel = {0, 0, 0}; // Declare the sum of calibration offsets to be used for calculating the bias

  vec3_t accelBias = {0, 0, 0}; // Declare the calibrated bias of the accelerometer to be subtracted from readings

  Accelerometer(int csPin, vec3_t offset, int axesInfo[6]) : csPin(csPin), offset(offset / 100), axesInfo(axesInfo) {
    pinMode(csPin, OUTPUT); // Set up the chip select pin to be output
    digitalWrite(csPin, HIGH); // Set chip select pin to high before communication starts
  }

  void initialize() {
    writeRegister(csPin, dataFormatAddress, 0x00); // Put the accelerometer into +/-2g range by writing the value 0x00 to the data format register
    writeRegister(csPin, powerControlAddress, 0x08); // Put the accelerometer into measurement mode by writing 0x08 to the power control register
  }

  void startCalibration() {
    calibrationCount = 0; // Reset the calibration count int
    calibratedAccel = {0, 0, 0}; // Reset the calibrated acceleration sum
    accelBias = {0, 0, 0}; // Clear the acceleration bias
  }

  void updateCalibration() {
    calibrationCount++; // Increment the calibration count
    calibratedAccel += this->readAcceleration() - gravityAccelVector; // Increase the calibration sum by the accelerometer offset from expected acceleration
  }

  void endCalibration() {
    accelBias = calibratedAccel / (float)calibrationCount; // Set the acceleration bias to the average of all the acceleration offsets
  }

  vec3_t readAcceleration() {
    readRegister(csPin, accelDataFirstAddress, accelerationDataByteCount); // Read raw acceleration data bytes

    rawAccel = { // Set the raw acceleration vector based on the data from the input buffer
      (inputBuffer[1] << 8) | inputBuffer[0], // Get raw x value from input buffer
      (inputBuffer[3] << 8) | inputBuffer[2], // Get raw y value from input buffer
      (inputBuffer[5] << 8) | inputBuffer[4] // Get raw z value from input buffer
    };

    for (j = 0; j < 3; j++) { // Loop through acceleration axes
      formattedAccel.set(j, rawAccel.get(axesInfo[j]) * (float)axesInfo[3 + j]); // Set formatted acceleration for each axis based on the axis and sign conversions in the axes info array
    }

    return formattedAccel * gravityAccelMagnitude / accelerometerReadingScale - accelBias; // Divide acceleration by g scale, multiply by gravity magnitude and subtract bias
  }
};

// Declare a struct for holding button pin, states and reading methods
struct Button {
  int pin; // Declare the button pin number
  bool pressState = false; // Declare the current press state of the button (true = pressed, false = not pressed)
  bool previousState = false; // Declare the previous press state of the button

  Button(int pin) : pin(pin) {
    pinMode(pin, INPUT); // Set the pin to input mode for reading
  }

  void updateState() {
    previousState = pressState; // Set the previous press state to the current state
    pressState = digitalRead(pin) == HIGH; // Set the press state to true if the read signal for the pin is high
  }

  bool held() {
    return pressState; // Return the current press state
  }

  bool pressed() {
    return previousState == false && pressState == true; // Return true if the button was previously unpressed and is now pressed
  }

  bool released() {
    return previousState == true && pressState == false; // Return true if the button was previously pressed and is now unpressed
  }
};

vec3_t offset1 = {0.3, -5.8, 6}; // Declare the vector offset from the sensor of the first accelerometer
vec3_t offset2 = {0.3, -5.8, 3}; // Declare the vector offset from the sensor of the second accelerometer
vec3_t offset3 = {-5, -5.8, 1.2}; // Declare the vector offset from the sensor of the third accelerometer

int axesInfo1[6] = {0, 2, 1, -1, -1, 1}; // Declare the acceleration axes and sign formats for the first accelerometer
int axesInfo2[6] = {0, 2, 1, 1, -1, -1}; // Declare the acceleration axes and sign formats for the second accelerometer
int axesInfo3[6] = {0, 2, 1, -1, -1, 1}; // Declare the acceleration axes and sign formats for the third accelerometer

Accelerometer accelerometers[3] = { // Declare the array of the 3 accelerometers
  {2, offset1, axesInfo1}, // Construct the first accelerometer with chip select pin 2
  {5, offset2, axesInfo2}, // Construct the second accelerometer with chip select pin 5
  {3, offset3, axesInfo3} // Construct the third accelerometer with chip select pin 3
};

const vec3_t scanDirection = {0, 0, -1}; // Define the scan/facing direction of the ultrasonic sensor
Ultrasonic ultrasonicSensor(6, 12); // Declare the accelerometer with trigger pin 6 and echo pin 12

Button resetButton(11); // Declare the reset button with pin 11
Button measureButton(13); // Declare the measure button with pin 13

const int blackColor = 0; // Declare the int for drawing the color black on the epd display
const int whiteColor = 1; // Declare the int for drawing the color white on the epd display
const int screenWidth = 296; // Declare the screen width of the epd display
const int screenHeight = 128; // Declare the screen height of the epd display
const float aspectRatio = (float)screenWidth / (float)screenHeight; // Declare the aspect ratio of the epd display

Epd epd; // Declare the epd
unsigned char image[4736]; // Declare the image buffer
Paint paint(image, screenWidth, screenHeight); // Declare the paint object with the image buffer and display dimensions

long lastTime = 0; // Declare the time from the previous loop in milliseconds
float deltaTime; // Declare the time passed since the last loop in seconds
long currentTime; // Declare the current time of the current loop in milliseconds

vec3_t worldAcceleration; // Declare the world acceleration of the accelerometer
vec3_t worldOffset; // Declare the world offset from the origin of the accelerometer
vec3_t output; // Declare the output vector for the linear system

vec3_t position = {0, 0, 0}; // Declare the origin position
vec3_t velocity = {0, 0, 0}; // Declare the origin velocity
vec3_t acceleration; // Declare the origin acceleration

quat_t orientation = {1, 0, 0, 0}; // Declare the quaternion orientation of the origin
vec3_t angularVelocity = {0, 0, 0}; // Declare the angular velocity of the origin
vec3_t angularAcceleration; // Declare the angular acceleration of the origin

vec3_t rotation; // Declare the rotation of the origin for a given loop
float rotationMagnitude; // Declare the magnitude of the rotation
quat_t rotationQuaternion; // Declare the rotation in quaternion form to be multiplied to the orientation

float scanDistance; // Declare the scan distance in meters detected by the ultrasonic sensor
vec3_t measuredPoint; // Declare the measured point using the scanned distance
vec3_t measuredVector; // Declare the measured vector between two measured points

int pointCount = 0; // Declare the amount of points measured
vec3_t measuredPoints[2]; // Declare the array to store the measured points

int vectorCount = 0; // Declare the amount of vectors measured
vec3_t measuredVectors[3]; // Declare the array to store the measured vectors

float distance; // Declare the distance of one measured vectors
float area; // Declare the area formed by two measured vectors
float volume; // Declare the volume formed by three measured vectors

const BLA::Matrix<3, 3> identityMatrix = { // Declare the 3x3 identity matrix
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};

BLA::Matrix<9, 6> coefficientMatrix; // Declare the coefficient matrix for the translation and angular acceleration linear system
BLA::Matrix<6, 1> unknownVector; // Declare the vector storing the unknown translation and angular accelerations of the linear system
BLA::Matrix<9, 1> outputVector; // Declare the known component of the linear system outputted when the coefficient matrix is applied to the unknown vector

BLA::Matrix<3, 3> getCrossMatrix(vec3_t vector) {
  return { // Return the matrix which applies a cross product operation between the vector it is multiplied with and the inputted vector parameter
    0, -vector.z, vector.y,
    vector.z, 0, -vector.x,
    -vector.y, vector.x, 0
  };
}

const int maxScans = 10; // Declare the max amount of points stored and displayed at a given time
vec3_t scannedPoints[maxScans]; // Declare the array of scanned points
int scanIndex = 0; // Declare the current index in the scanned points array
int scanCount = 0; // Declare the amount of points scanned

void createScannedPoint(vec3_t point) {
  scannedPoints[scanIndex] = point; // Add the point to the scanned points array
  scanIndex++; // Increment the index for the array
  scanCount = min(scanCount + 1, maxScans); // Increment the point count and ensure it does not exceed the maximum amount of points

  if (scanIndex > maxScans) { // Check if the scan index has exceeded the array size
    scanIndex = 0; // Reset the scan index back to the start
  }
}

const float fieldOfView = 70; //
const float fieldOfViewFactor = 1 / tan((3.14159 * fieldOfView / 180) / 2);

const vec3_t normalizingVector = {1, 1, 1}; // Declare the normalizing vector to add to the screen projection to ensure it normalize it between 0 to 1
vec3_t cameraSpacePoint; // Declare the point position relative to the camera's position and orientation in space

const float defaultPointSize = 1; // Declare the size of measured points on the screen
bool shouldOcclude; // Declare whether or not the point is outside of the view frustum or not
int screenX; // Declare the x pixel position of the point on the screen
int screenY; // Declare the y pixel position of the point on the screen
int pointSize; // Declare the pixel size of the point on the screen

void projectPointOnScreen(vec3_t point, bool& occlude, int& x, int& y, int& size) {
  occlude = false; // Set occlude to false
  cameraSpacePoint = orientation.conj().rotate(point - position, true); // Convert the point to camera space by subtracting the origin position and applying the inverse orientation quaternion

  if (cameraSpacePoint.z >= 0) { // Check if the point is behind the camera
    occlude = true; // Set occlude to true

    return; // Break the function
  }

  size = defaultPointSize / -cameraSpacePoint.z; // Scale the point size by the distance from the camera for a perspective effect

  cameraSpacePoint /= -cameraSpacePoint.z; // Divide the point by the distance from the camera to obtain the perspective effect
  cameraSpacePoint *= fieldOfViewFactor; // Multiply the perspective scaled camera position by the field of view factor
  cameraSpacePoint.set(0, cameraSpacePoint.get(0) / aspectRatio); // Divide the x component by the aspect ratio to account for the screen proportions

  if (cameraSpacePoint.x < -1 || cameraSpacePoint.x > 1 || cameraSpacePoint.y < -1 || cameraSpacePoint.y > 1) { // Check if the camera space point is outside the view frustum
    occlude = true; // Set occlude to true

    return; // Break the function
  }

  cameraSpacePoint = (cameraSpacePoint + normalizingVector) / 2; // Normalize the vector to be between 0 and 1

  x = (int)(cameraSpacePoint.x * (float)screenWidth); // Set x based on the normalized camera space point where 0 = 0 and 1 = the screen width
  y = screenHeight - (int)(cameraSpacePoint.y * (float)screenHeight); // Set y based on the normalized camera space point where 0 = the screen height and 1 = 0 (so that it starts at the bottom corner)
}

String displayMessage; // Declare the concatenated string display message for epd text displaying
char stringBuffer[24]; // Declare the char buffer for displaying concatenated string messages

void drawLabeledValue(String label, float value, String unit, int order) {
  displayMessage = label + ": " + String(value) + unit; // Concatenate the label, value and unit parameters into the display message
  displayMessage.toCharArray(stringBuffer, sizeof(stringBuffer)); // Fill the char buffer with the concatenated string message
  
  paint.DrawStringAt(0, order * 12, stringBuffer, &Font12, blackColor); // Draw the string to the paint object at its specified row
}

void updateDisplay() {
  paint.SetRotate(ROTATE_90); // Rotate the paint by 90 degrees so that the x component and width represent the longer side's dimension
  paint.SetWidth(screenHeight); // Set the width as the screen height (treated as unrotated)
  paint.SetHeight(screenWidth); // Set the height as the screen width (treated as unrotated)
 
  paint.Clear(whiteColor); // Clear the canvas with white color
 
  if (vectorCount >= 1) { // Check if there is at least one measured vector
    drawLabeledValue("Distance", distance, "m", 0); // Display the distance of the measured vector
  }

  if (vectorCount >= 2) { // Check if there are at least two measured vectors
    drawLabeledValue("Area", area, "m^2", 1); // Display the area of the two measured vectors
  }

  if (vectorCount >= 3) { // Check if there are at least three measured vectors
    drawLabeledValue("Volume", distance, "m^3", 2); // Display the volume of the three measured vectors
  }

  for (i = 0; i < scanCount; i++) { // Loop through the scanned points
    projectPointOnScreen(scannedPoints[i], shouldOcclude, screenX, screenY, pointSize); // Project the point to get its x, y and size ints, and whether or not its out of view

    if (!shouldOcclude) { // Check if the point should not be occluded (in view)
      paint.DrawFilledCircle(screenX, screenY, pointSize, blackColor); // Draw the point
    }
  }

  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight()); // Update the display with the new pixel information
  epd.DisplayFrame(); // Display the changes on the screen
}

void setup(){
  Serial.begin(9600); // Create a serial connection to display the data on the terminal

  SPI.begin(); // Initiate an SPI communication instance
  SPI.setDataMode(SPI_MODE3); // Configure the SPI connection for the ADXL345

  for (int i = 0; i < 3; i++) { // Loop through the three leftmost 3x3 submatrices of the coefficient matrix
    coefficientMatrix.Submatrix<3, 3>(i * 3, 0) = identityMatrix; // Fill the 3x3 submatrix with the identity matrix
  }

  for (int i = 0; i < 3; i++) { // Loop through the accelerometers
    accelerometers[i].initialize(); // Initialize the accelerometer
  }

  if (epd.Init(lut_partial_update) != 0) { // Set the epd mode to partial updates and ensure it initializes properly
    Serial.print("e-Paper init failed");

    return;
  }

  epd.ClearFrameMemory(0xFF); // Clear the frame memory of the display
  epd.DisplayFrame(); // Display the changes to the display
  epd.ClearFrameMemory(0xFF); // Clear the frame memory of the display again to make up for partial updates
  epd.DisplayFrame(); // Display the changes to the display to make up for partial updates
}

void loop(){
  currentTime = millis(); // Set the current loop time
  deltaTime = (float)(currentTime - lastTime) / 1000; // Calculate the time passed in seconds with the difference in time between the current and last loop
  lastTime = currentTime; // Set the last time as the current loop time to be used for the next loop

  resetButton.updateState(); // Update the state of the reset button

  if (resetButton.pressed()) { // Check if the reset button was pressed
    for (i = 0; i < 3; i++) { // Loop through the accelerometers
      accelerometers[i].startCalibration(); // Start the calibration of the accelerometer
    }

    position = {0, 0, 0}; // Reset the origin position
    velocity = {0, 0, 0}; // Reset the origin velocity
    orientation = {1, 0, 0, 0}; // Reset the origin orientation
    angularVelocity = {0, 0, 0}; // Reset the origin angular velocity

    Serial.println("Calibrating"); // Print the calibration start message to the serial monitor

  } else if (resetButton.released()) { // Check if the reset button was released
    for (i = 0; i < 3; i++) { // Loop through the accelerometers
      accelerometers[i].endCalibration(); // End the accelerometer's calibration
    }

    Serial.println("Calibrated"); // Print the calibration end message to the serial monitor
  }

  if (resetButton.held()) { // Check if the reset button is being held
    for (i = 0; i < 3; i++) { // Loop through the accelerometers
      accelerometers[i].updateCalibration(); // Update the accelerometer's calibration
    }
   
    return; // Exit before the rest of the loop
  }

  for (i = 0; i < 3; i++) { // Loop through the accelerometers
    worldAcceleration = orientation.rotate(accelerometers[i].readAcceleration(), true) - gravityAccelVector; // Read the accelerometer's acceleration and convert it to world space
    worldOffset = orientation.rotate(accelerometers[i].offset, true); // Convert the accelerometer offset to world space
   
    output = worldAcceleration - angularVelocity.cross(angularVelocity.cross(worldOffset)); // Set the output vector as the known component of the linear system

    coefficientMatrix.Submatrix<3, 3>(i * 3, 3) = -getCrossMatrix(worldOffset); // Fill the coefficient matrix with the negative cross matrix of the accelerometer offset
    outputVector.Submatrix<3, 1>(i * 3, 0) = BLA::Matrix<3, 1>({output.x, output.y, output.z}); // Fill the output vector with the known output
  }

  unknownVector = BLA::Inverse(~coefficientMatrix * coefficientMatrix) * (~coefficientMatrix * outputVector); // Solve for unknown vector using least squares method

  acceleration = {unknownVector(0, 0), unknownVector(1, 0), unknownVector(2, 0)}; // Set acceleration as the first 3 columns of the unknown vector
  angularAcceleration = {unknownVector(3, 0), unknownVector(4, 0), unknownVector(5, 0)}; // Set the angular acceleration as the last 3 columns of the unknown vector

  position += velocity * deltaTime + acceleration * deltaTime * deltaTime / 2; // Get the change in position using the kinematic equation
  velocity += acceleration * deltaTime; // Increase the velocity by the acceleration
 
  rotation = angularVelocity * deltaTime + angularAcceleration * deltaTime * deltaTime / 2; //  Get the rotation change using the kinematic equation
  rotationMagnitude = rotation.mag(); // Get the magnitude of the rotation

  if (rotationMagnitude > 0) { // Check if the rotation magnitude is greater than zero
    rotationQuaternion.setRotation(rotation / rotationMagnitude, rotationMagnitude); // Define the rotation quaternion as a rotation around the rotation unit vector with angle equal to magnitude
   
    orientation *= rotationQuaternion; // Apply the rotation quaternion to the origin orientation
    orientation = orientation.norm(); // Ensure the origin orientation is normalized
  }
 
  angularVelocity += angularAcceleration * deltaTime; // Increase the angular velocity by the angular acceleration

  measureButton.updateState(); // Update the measure button's state

  if (measureButton.pressed()) { // Check if the measure button was pressed
    Serial.println("Pressed"); // Print the measure press message to the serial monitor

    scanDistance = (float)ultrasonicSensor.read() / 100; // Read the scan distance of the ultrasonic sensor (in centimeters) and convert to meters
    Serial.println(scanDistance);
    measuredPoint = position + orientation.rotate(scanDirection, true) * scanDistance; // Find the measured point using the origin position, look vector and scan distance

    measuredPoints[pointCount] = measuredPoint; // Add the measured point to the measured points array
    pointCount++; // Increment the point count

    if (pointCount == 2) { // Check if there are two points
      measuredVector = measuredPoints[1] - measuredPoints[0]; // Get the measured vector (difference between two points)
      pointCount = 0; // Reset the point count

      if (vectorCount == 3) { // Check if there are already 3 measured vectors
        vectorCount = 0; // Reset to zero measured vectors
      }

      measuredVectors[vectorCount] = measuredVector; // Fill the measured vectors array with the new measured vector
      vectorCount++; // Increment the measured vectors count

      if (vectorCount >= 1) { // Check if there is at least one measured vector
        Serial.println("Distance"); // Print the distance message to the serial monitor for debugging
       
        distance = measuredVectors[vectorCount - 1].mag(); // Calculate the distance using the magnitude of the last measured vector
      }

      if (vectorCount >= 2) { // Check if there are at least two measured vectors
        Serial.println("Area"); // Print the area message to the serial monitor for debugging
       
        area = measuredVectors[vectorCount - 2].cross(measuredVectors[vectorCount - 1]).mag(); // Calculate the area of the parallelogram between the last two vectors using the magnitude of the cross product
      }

      if (vectorCount >= 3) { // Check if there are at least three measured vectors
        Serial.println("Volume"); // Print the volume message to the serial monitor for debugging
       
        volume = abs(measuredVectors[0].dot(measuredVectors[1].cross(measuredVectors[2]))); // Calculate the volume of the parallelepiped formed by the three vectors using the scalar triple product
      }
    }

    createScannedPoint(measuredPoint); // Add the measured point to the scanned points to be displayed
    updateDisplay(); // Update the epd display to show the distance/area/volume/scanned point changes
  }
}