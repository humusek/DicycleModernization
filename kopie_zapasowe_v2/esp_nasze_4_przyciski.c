#include <Bluepad32.h>

//My Variables
int axisX, throttle, brake;
bool buttonA = false;
bool buttonB = false;
bool buttonX = false;
bool buttonY = false;

bool isStarted = false;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            //Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            //Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        //Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            //Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        //Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    axisX = ctl->axisX();
    throttle = ctl->throttle();
    brake = ctl->brake();
    
    // Odczyt stanów przycisków
    buttonA = ctl->a();
    buttonB = ctl->b();
    buttonX = ctl->x();
    buttonY = ctl->y();
}

void processGamepad(ControllerPtr ctl) {
    dumpGamepad(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    const uint8_t* addr = BP32.localBdAddress();
    BP32.setup(&onConnectedController, &onDisconnectedController);

    BP32.forgetBluetoothKeys();
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    if (abs(axisX) < 30) {
        axisX = 0;
    }

    //Serial.printf("AxisX: %4d, Throttle: %4d, Brake: %4d \n", axisX, throttle, brake);
    //0000:1023:0000;
    
    // Wysyłanie danych w formacie: OśX:Gaz:Hamulec:PrzyciskA:PrzyciskB:PrzyciskX:PrzyciskY;
    Serial.printf("%4d:%4d:%4d:%d:%d:%d:%d;", axisX, throttle, brake, buttonA, buttonB, buttonX, buttonY);
    //Serial.printf("%4d:%4d:%4d:%d:%d:%d:%d;\n", axisX, throttle, brake, buttonA, buttonB, buttonX, buttonY); //test
    delay(10);
}