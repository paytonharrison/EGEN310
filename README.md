# EGEN310
The CONTROL APP code and Adafruit 32u4 code for controlling an RC car.

The Adafruit Bluefruit uses BLE to connect to the phone application. The board was programmed using Arduino sketches, and uses parts of the Arduino DC Motor Test example. Our car is 4 wheel drive, and uses 4 motors, so we also used an Adafruit DC Motor and Stepper FeatherWing.

Our car uses skid steer, so to turn left or right, we just power half of the motors forward and half backward. In terms of the application UI, this meant a control pad would work just fine and still be intuitive/functional. The app was designed in MIT App Inventor. The only way to export that code is via a .apk, so screenshots of the app UI and some of the coding blocks have also been included.

The "EGEN_Car" folder is the Arduino sketch code, and the "EGEN_Car.apk" is the MIT App Inventor package.

