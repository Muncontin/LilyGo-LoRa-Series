/*
   RadioLib Transmit with Interrupts Example

   This example transmits packets using SX1276/SX1278/SX1262/SX1268/SX1280/LR1121 LoRa radio module.
   Each packet contains up to 256 bytes of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

#include "LoRaBoards.h"
#include <RadioLib.h>
#include <mqttservice.h>
#include <button_handler.h>
#include <loramesh.h>
#define BUTTON_PIN 38

#if     defined(USING_SX1276)
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           868.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   17
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif
SX1276 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);

#elif   defined(USING_SX1278)
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           433.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   17
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif
SX1278 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);

#elif   defined(USING_SX1262)
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           923.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   16 //og 16
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0 //og 125.0
#endif

SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

#elif   defined(USING_SX1280)
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           2400.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   13
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             203.125
#endif
SX1280 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

#elif  defined(USING_SX1280PA)
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           2400.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   3           // PA Version power range : -18 ~ 3dBm
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             203.125
#endif
SX1280 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

#elif   defined(USING_SX1268)
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           433.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   22
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif
SX1268 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

#elif   defined(USING_LR1121)

// The maximum power of LR1121 2.4G band can only be set to 13 dBm
// #ifndef CONFIG_RADIO_FREQ
// #define CONFIG_RADIO_FREQ           2450.0
// #endif
// #ifndef CONFIG_RADIO_OUTPUT_POWER
// #define CONFIG_RADIO_OUTPUT_POWER   13
// #endif
// #ifndef CONFIG_RADIO_BW
// #define CONFIG_RADIO_BW             125.0
// #endif

// The maximum power of LR1121 Sub 1G band can only be set to 22 dBm
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           868.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   22
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif

LR1121 radio = new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

// save transmission state between loops
static int transmissionState = RADIOLIB_ERR_NONE;
static volatile bool transmittedFlag = false;
static volatile bool receivedFlag = false;
static uint32_t counter = 10000;
static String payload;
static String deviceID;
static String rssi = "0dBm";
static String snr = "0dB";
int currentSF = 7; // Default to SF7
unsigned long lastRDPTime = 0; //Route Discovery packet
const unsigned long RDP_INTERVAL = 30000;  // 30s interval

#define MAX_NEIGHBORS 20  // Max number of tracked neighbors

struct NeighborEntry {
    String neighborID;
    int bestSF;
};

NeighborEntry neighborTable[MAX_NEIGHBORS];
int neighborCount = 0;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void)
{
    // we sent a packet, set the flag
    transmittedFlag = true;
}

void setReceivedFlag(void) 
{
    receivedFlag = true;
}

String idGenerator() {
    uint64_t chipId = ESP.getEfuseMac();  // Get ESP32's unique MAC address
    return "ttgo-" + String((uint32_t)(chipId >> 32), HEX) + String((uint32_t)chipId, HEX);
}

void setup()
{
    setupBoards();
    mqttsetup();
      // Setup the button pin and attach the interrupt
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Assuming active LOW button with pull-up resistor
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, FALLING);

    deviceID = idGenerator();
    Serial.println("Generated device ID: " + deviceID);

    // When the power is turned on, a delay is required.
    delay(1500);

#ifdef  RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);
    digitalWrite(RADIO_TCXO_ENABLE, HIGH);
#endif

    // initialize radio with default settings
    int state = radio.begin();

    printResult(state == RADIOLIB_ERR_NONE);

    Serial.print(F("Radio Initializing ... "));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // set the function that will be called
    // when packet transmission is finished
    radio.setPacketSentAction(setFlag);

    // set the function that will be called
    // when new packet is received
    radio.setPacketReceivedAction(setReceivedFlag);


    /*
    *   Sets carrier frequency.
    *   SX1278/SX1276 : Allowed values range from 137.0 MHz to 525.0 MHz.
    *   SX1268/SX1262 : Allowed values are in range from 150.0 to 960.0 MHz.
    *   SX1280        : Allowed values are in range from 2400.0 to 2500.0 MHz.
    *   LR1121        : Allowed values are in range from 150.0 to 960.0 MHz, 1900 - 2200 MHz and 2400 - 2500 MHz. Will also perform calibrations.
    * * * */

    if (radio.setFrequency(CONFIG_RADIO_FREQ) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        Serial.println(F("Selected frequency is invalid for this module!"));
        while (true);
    }

    /*
    *   Sets LoRa link bandwidth.
    *   SX1278/SX1276 : Allowed values are 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz. Only available in %LoRa mode.
    *   SX1268/SX1262 : Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
    *   SX1280        : Allowed values are 203.125, 406.25, 812.5 and 1625.0 kHz.
    *   LR1121        : Allowed values are 62.5, 125.0, 250.0 and 500.0 kHz.
    * * * */
    if (radio.setBandwidth(CONFIG_RADIO_BW) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        Serial.println(F("Selected bandwidth is invalid for this module!"));
        while (true);
    }


    /*
    * Sets LoRa link spreading factor.
    * SX1278/SX1276 :  Allowed values range from 6 to 12. Only available in LoRa mode.
    * SX1262        :  Allowed values range from 5 to 12.
    * SX1280        :  Allowed values range from 5 to 12.
    * LR1121        :  Allowed values range from 5 to 12.
    * * * */
    if (radio.setSpreadingFactor(7) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) { //Change this to determine distance/speed etc.
        Serial.println(F("Selected spreading factor is invalid for this module!"));
        while (true);
    }

    /*
    * Sets LoRa coding rate denominator.
    * SX1278/SX1276/SX1268/SX1262 : Allowed values range from 5 to 8. Only available in LoRa mode.
    * SX1280        :  Allowed values range from 5 to 8.
    * LR1121        :  Allowed values range from 5 to 8.
    * * * */
    if (radio.setCodingRate(6) == RADIOLIB_ERR_INVALID_CODING_RATE) {
        Serial.println(F("Selected coding rate is invalid for this module!"));
        while (true);
    }

    /*
    * Sets LoRa sync word.
    * SX1278/SX1276/SX1268/SX1262/SX1280 : Sets LoRa sync word. Only available in LoRa mode.
    * * */
    if (radio.setSyncWord(0xAB) != RADIOLIB_ERR_NONE) {
        Serial.println(F("Unable to set sync word!"));
        while (true);
    }

    /*
    * Sets transmission output power.
    * SX1278/SX1276 :  Allowed values range from -3 to 15 dBm (RFO pin) or +2 to +17 dBm (PA_BOOST pin). High power +20 dBm operation is also supported, on the PA_BOOST pin. Defaults to PA_BOOST.
    * SX1262        :  Allowed values are in range from -9 to 22 dBm. This method is virtual to allow override from the SX1261 class.
    * SX1268        :  Allowed values are in range from -9 to 22 dBm.
    * SX1280        :  Allowed values are in range from -18 to 13 dBm. PA Version range : -18 ~ 3dBm
    * LR1121        :  Allowed values are in range from -17 to 22 dBm (high-power PA) or -18 to 13 dBm (High-frequency PA)
    * * * */
    if (radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        Serial.println(F("Selected output power is invalid for this module!"));
        while (true);
    }

#if !defined(USING_SX1280) && !defined(USING_LR1121) && !defined(USING_SX1280PA)
    /*
    * Sets current limit for over current protection at transmitter amplifier.
    * SX1278/SX1276 : Allowed values range from 45 to 120 mA in 5 mA steps and 120 to 240 mA in 10 mA steps.
    * SX1262/SX1268 : Allowed values range from 45 to 120 mA in 2.5 mA steps and 120 to 240 mA in 10 mA steps.
    * NOTE: set value to 0 to disable overcurrent protection
    * * * */
    if (radio.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        Serial.println(F("Selected current limit is invalid for this module!"));
        while (true);
    }
#endif

    /*
    * Sets preamble length for LoRa or FSK modem.
    * SX1278/SX1276 : Allowed values range from 6 to 65535 in %LoRa mode or 0 to 65535 in FSK mode.
    * SX1262/SX1268 : Allowed values range from 1 to 65535.
    * SX1280        : Allowed values range from 1 to 65535. preamble length is multiple of 4
    * LR1121        : Allowed values range from 1 to 65535.
    * * */
    if (radio.setPreambleLength(16) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
        Serial.println(F("Selected preamble length is invalid for this module!"));
        while (true);
    }

    // Enables or disables CRC check of received packets.
    if (radio.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        Serial.println(F("Selected CRC is invalid for this module!"));
        while (true);
    }

#if  defined(USING_LR1121)
    // LR1121
    // set RF switch configuration for Wio WM1110
    // Wio WM1110 uses DIO5 and DIO6 for RF switching
    static const uint32_t rfswitch_dio_pins[] = {
        RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
        RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
    };

    static const Module::RfSwitchMode_t rfswitch_table[] = {
        // mode                  DIO5  DIO6
        { LR11x0::MODE_STBY,   { LOW,  LOW  } },
        { LR11x0::MODE_RX,     { HIGH, LOW  } },
        { LR11x0::MODE_TX,     { LOW,  HIGH } },
        { LR11x0::MODE_TX_HP,  { LOW,  HIGH } },
        { LR11x0::MODE_TX_HF,  { LOW,  LOW  } },
        { LR11x0::MODE_GNSS,   { LOW,  LOW  } },
        { LR11x0::MODE_WIFI,   { LOW,  LOW  } },
        END_OF_MODE_TABLE,
    };
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

    // LR1121 TCXO Voltage 2.85~3.15V
    radio.setTCXO(3.0);
#endif

#ifdef USING_DIO2_AS_RF_SWITCH
#ifdef USING_SX1262
    // Some SX126x modules use DIO2 as RF switch. To enable
    // this feature, the following method can be used.
    // NOTE: As long as DIO2 is configured to control RF switch,
    //       it can't be used as interrupt pin!
    if (radio.setDio2AsRfSwitch() != RADIOLIB_ERR_NONE) {
        Serial.println(F("Failed to set DIO2 as RF switch!"));
        while (true);
    }
#endif //USING_SX1262
#endif //USING_DIO2_AS_RF_SWITCH

#ifdef RADIO_RX_PIN
    // SX1280 PA Version
    radio.setRfSwitchPins(RADIO_RX_PIN, RADIO_TX_PIN);
#endif

#ifdef RADIO_SWITCH_PIN
    // T-MOTION
    const uint32_t pins[] = {
        RADIO_SWITCH_PIN, RADIO_SWITCH_PIN, RADIOLIB_NC,
    };
    static const Module::RfSwitchMode_t table[] = {
        {Module::MODE_IDLE,  {0,  0} },
        {Module::MODE_RX,    {1, 0} },
        {Module::MODE_TX,    {0, 1} },
        END_OF_MODE_TABLE,
    };
    radio.setRfSwitchTable(pins, table);
#endif




    // start transmitting the first packet
    Serial.print(F("Radio Sending first packet ... "));

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    //transmissionState = radio.startTransmit(String(counter).c_str());
    broadcastRouteUpdate();
    printNeighbors();

    // you can also transmit byte array up to 256 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      state = radio.startTransmit(byteArr, 8);
    */
    delay(1000);

}

void printNeighbors() {
    Serial.println("\nNeighbor List:");
    Serial.println("-----------------------------------");
    Serial.println("Neighbor ID       | SF ");
    Serial.println("-----------------------------------");

    if (neighborCount == 0) {
        Serial.println("No neighbors discovered yet.");
        Serial.println("-----------------------------------\n");
        return;
    }

    for (int i = 0; i < neighborCount; i++) {
        Serial.print(neighborTable[i].neighborID);
        Serial.print("  | SF");
        Serial.println(neighborTable[i].bestSF);
    }

    Serial.println("-----------------------------------\n");
}


void broadcastRouteUpdate() {
    // Periodically sends a routing discovery packet to map out the mash network
    //RDP|<SenderID>,<SF>|<Destination1,Distance1,SF1>|<Destination2,Distance2,SF2>|...

    int bestSF = findBestNeighborSF();  // Select the best SF from known neighbors

    if (bestSF == -1) bestSF = 7;  // Default to SF7 if no neighbors exist

    radio.setSpreadingFactor(bestSF);  // Set the best SF
    currentSF = bestSF;

    Serial.print("Broadcasting RDP at SF");
    Serial.println(bestSF);

    // Advertise the existence of this node to all neighbors
    String routeMessage = "RDP|" + deviceID + "," + String(bestSF);

    // for (int i = 0; i < routeCount; i++) {
    //     routeMessage += "|" + routingTable[i].destination + "," + 
    //                     String(routingTable[i].distance) + "," + 
    //                     String(routingTable[i].spreadingFactor);
    // }

    sendLoraMessage(routeMessage);
}

int findBestNeighborSF() {
    if (neighborCount == 0) return -1;  // No neighbors stored yet

    int bestSF = 12;  // Start with the highest SF (worst case)

    for (int i = 0; i < neighborCount; i++) {
        if (neighborTable[i].bestSF < bestSF) {
            bestSF = neighborTable[i].bestSF;
        }
    }

    return bestSF;
}

void processRouteUpdate(String payload) {
    if (!payload.startsWith("RDP|")) return; // Ignore non-RDP messages

    int commaPos = payload.indexOf(",", 4);
    if (commaPos == -1) {
        Serial.println("Error: Invalid RDP format (missing SF).");
        return;
    }

    // Extract sender ID and SF
    String senderID = payload.substring(4, commaPos);
    int senderSF = payload.substring(commaPos + 1).toInt();

    // Ignore messages from itself
    if (senderID == deviceID) {
        Serial.println("Ignoring self-received RDP.");
        return;
    }

    if (senderSF < 7 || senderSF > 12) { // Ensure SF is within valid range
        Serial.println("Error: Invalid SF value in RDP message.");
        return;
    }

    // Save/update SF for this neighbor
    updateNeighborSF(senderID, senderSF);

    Serial.print("Updated SF for ");
    Serial.print(senderID);
    Serial.print(" to SF");
    Serial.println(senderSF);
}


// Updates a routing table
void updateNeighborSF(String neighborID, int sf) {
    for (int i = 0; i < neighborCount; i++) {
        if (neighborTable[i].neighborID == neighborID) {
            // Update SF if different
            if (neighborTable[i].bestSF != sf) {
                neighborTable[i].bestSF = sf;
                Serial.print("Updated stored SF for ");
                Serial.print(neighborID);
                Serial.print(" to SF");
                Serial.println(sf);
            }
            return;
        }
    }

    // Add new neighbor if space available
    if (neighborCount < MAX_NEIGHBORS) {
        neighborTable[neighborCount].neighborID = neighborID;
        neighborTable[neighborCount].bestSF = sf;
        neighborCount++;
        Serial.print("Added new neighbor ");
        Serial.print(neighborID);
        Serial.print(" with SF");
        Serial.println(sf);
    } else {
        Serial.println("Neighbor table full, cannot add new neighbor.");
    }
}

//Adjust spreading factor based on the RSSI and SNR values
void adjustSpreadingFactor(float rssiValue, float snrValue) {

    // Select SF based on RSSI & SNR
    if (rssiValue < -125 || snrValue < 0) {
        currentSF = 12; // Very weak signal, maximize range but slowest
    } else if (rssiValue < -120 || snrValue < 1) {
        currentSF = 11; // Weak signal, increase SF for stability
    } else if (rssiValue < -110 || snrValue < 2) {
        currentSF = 10; // Moderate signal, some noise
    } else if (rssiValue < -100 || snrValue < 4) {
        currentSF = 9; // Medium range, good balance
    } else if (rssiValue < -90 || snrValue < 6) {
        currentSF = 8; // Strong link, optimize for speed
    } else {
        currentSF = 7; // Very strong, fastest transmission
    }

    // Apply the new spreading factor dynamically
    if (radio.setSpreadingFactor(currentSF) == RADIOLIB_ERR_NONE) {
        Serial.print("Adjusted Spreading Factor to SF");
        Serial.println(currentSF);
    } else {
        Serial.println("SF adjustment failed!");
    }
}

// Function to send a LoRa message "hello from ttgo"
void sendLoraMessage(String message) {
    
    // Reset transmitted flag before sending
    transmittedFlag = false;
    
    // Flash an LED to indicate activity (if implemented)
    flashLed();
    
    // Start transmission with the given payload
    int transmissionState = radio.startTransmit(message.c_str());
    
    if (transmissionState == RADIOLIB_ERR_NONE) {
      Serial.println(F("Transmission finished!"));
      Serial.print(F("Sent message:\t\t"));
      Serial.println(message);
      // Optionally, publish the payload over MQTT as well
      client.publish("ttgo/network", message.c_str());
    } else {
      Serial.print(F("Transmission failed, code "));
      Serial.println(transmissionState);
    }

    message = "";
    
    // Optionally add a delay before next transmission (if needed)
    delay(1000);
    //setReceivedFlag();
    radio.startReceive();
}

void loop()
{
    unsigned long currentTime = millis();

    // Send RDP only if the interval has passed (currently every 30s)
    if (currentTime - lastRDPTime >= RDP_INTERVAL) {
        lastRDPTime = currentTime;
        broadcastRouteUpdate();
        printNeighbors();
    }
  
    // Process button press if the flag is set
    if (buttonPressed) {
        buttonPressed = false;  // Clear the flag
        Serial.println("Button Pressed on IO38!");
        setFlag();
    }
  
    //MQTT
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // LoRa
    // check if the previous transmission finished
    if (transmittedFlag) {
        // Set payload to our desired message
        payload = deviceID + " | " + String(counter++);
        sendLoraMessage(payload);
    }

    if(receivedFlag) {
        // reset flag
        receivedFlag = false; //Ensures that the packet is only read once

        String incoming;

        // you can read received data as an Arduino String
        int state = radio.readData(incoming);

        // you can also read received data as byte array
        /*
            byte byteArr[8];
            int state = radio.readData(byteArr, 8);
        */

        flashLed();

        if (state == RADIOLIB_ERR_NONE) {

            float rssiValue = radio.getRSSI();
            float snrValue = radio.getSNR();
            rssi = String(rssiValue) + "dBm"; //Received signal strength indicator, always negative between -30dbm to -140dbm. closer to 0 menas a stronger signal
            snr = String(snrValue) + "dB"; //signal to noise ratio, higher means better quality 

            // Check if the message is completely empty
            if (incoming == "" || incoming.length() < 5) { 
                Serial.println("Ignoring empty or very short message");
                return;
            }

            // Check if the message contains non-printable characters
            bool isCorrupted = false;
            for (int i = 0; i < incoming.length(); i++) {
                char c = incoming.charAt(i);
                if (c < 32 || c > 126) { // ASCII printable range
                    isCorrupted = true;
                    break;
                }
            }

            if (isCorrupted) {
                Serial.println("Warning: Garbled message detected! Possible corruption.");
                Serial.print("Raw Data: ");
                Serial.println(incoming);
                return;
            }

            if (incoming.startsWith("RDP|")) {
                processRouteUpdate(incoming);
                return; //skip as its routing update
            }

            //String senderID = incoming.substring(incoming.indexOf("ttgo-"));
            String senderID = incoming.substring(0, 17); //Extract out the incoming TTGO ID

            if (senderID == deviceID) {
                Serial.println("Ignoring self-received message");
            } else {
                //adjustSpreadingFactor(rssiValue, snrValue); //TODO: Change instead of global sf change to per neighbor

                // packet was successfully received
                Serial.println(F("Radio Received packet!"));

                // print data of the packet
                Serial.print(F("Radio Data:\t\t"));
                Serial.println(incoming);

                // print RSSI (Received Signal Strength Indicator)
                Serial.print(F("Radio RSSI:\t\t"));
                Serial.println(rssi);

                // print SNR (Signal-to-Noise Ratio)
                Serial.print(F("Radio SNR:\t\t"));
                Serial.println(snr);

                //Publish to MQTT broker
                publishMessage(incoming);
            }

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            Serial.println(F("CRC error!"));
        } else {
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.println(state);
        }

        radio.startReceive();
    }
}


