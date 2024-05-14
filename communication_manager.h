#include "BluetoothSerial.h"
#include "string.h"
#include <Arduino.h>



#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

/**
 * @file
 * @brief Communication Manager class for handling Bluetooth communication.
 */
class CommunicationManager
{
private:
    BluetoothSerial SerialBT; /**< Bluetooth serial object */
public:

    /**
     * @brief Initializes the Bluetooth serial communication.
     * 
     * @details Begins the Bluetooth serial communication with the given device name.
     */
    CommunicationManager(){
            this->SerialBT.begin("PROMEESP32");
        }


   /**
    * @brief Sends data over Bluetooth serial communication.
    * 
    * @param data The data to be sent as a String.
    */
    void send_data(String data){
        for(unsigned int i = 0; i<data.length(); i++) {
        char c = data.charAt(i); 
        this->SerialBT.write(data.charAt(i));  
        }
    }


    /**
     * @brief Receives data over Bluetooth serial communication.
     * 
     * @return The received data as a String.
     */
    String receive_data(){
        String data = "";
        if (SerialBT.available()){
            data = SerialBT.readString();
        }
        return data;
    }

    /**
     * @brief Destructor for the CommunicationManager class.
     * 
     * @details Ends the Bluetooth serial communication.
     */
    ~CommunicationManager(){
        SerialBT.end();
    }

};

#endif

