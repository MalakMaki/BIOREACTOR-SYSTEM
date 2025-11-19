//Make sure to calibrate KpH against substances of known pH

const byte pHPin=A0;
const float KpH = 3.3/4095; // Assuming a potential divider is used to map 0-5 V to 0-3.3 V
float pH, pHsmoothed;
int Timems, T2;

void setup() {
pinMode(pHPin,INPUT); // Pins are inputs by default, but it is good practice to set them anyway!
Serial.begin(2000000);
}

void loop() {
pH=KpH*analogRead(pHPin);
pHsmoothed = 0.9*pHsmoothed + 0.1*pH; // IIR smoothing to reduce noise

Timems=millis();
if(Timems-T2>0) {T2 = T2 + 1000; // Print pH value once per second
Serial.println(pHsmoothed);}
}
