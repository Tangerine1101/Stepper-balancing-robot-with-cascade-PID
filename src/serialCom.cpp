#include "serialCom.h"
    serialCommunication::serialCommunication(uint8_t startByte)
{
    txPackage.startByte = startByte;
}

void serialCommunication::begin(){
    usbPort.begin(SERIAL_BAUD);
}

void serialCommunication::sendBinary(float data[Max_arguments]){
    for (int i = 0; i < Max_arguments; i++){
        txPackage.data[i] = data[i];
    }
    txPackage.checkSum = checkSum(txPackage);
    usbPort.write((uint8_t*)&txPackage, sizeof(txPackage));
}   

void serialCommunication::sendText(float data[Max_arguments]){
    for (int i = 0; i < Max_arguments; i++){
        usbPort.print(data[i]);
        usbPort.print(",");
    }
    usbPort.println();
}

uint8_t serialCommunication::checkSum(serialPackage s){
    uint8_t* ptr = (uint8_t*)&s;
    uint8_t sum = 0;
    for (int i = 0; i < sizeof(s) - 1; i++){
        sum ^= ptr[i];
    }
    return sum;
}

void serialCommunication::readFrom(unsigned int pos, String Command){
    unsigned int startIndex = pos;
    unsigned int i = 0;
    int spaceIndex = -1;
    int hyphenIndex = -1;

    // Safety: Clear old arguments first so we don't get ghost values
    clearArgument(); 

    while(startIndex < Command.length() && i < Max_arguments){ 
        // 1. Find the hyphen
        hyphenIndex = Command.indexOf('-', startIndex);
        if(hyphenIndex == -1) break; // No more tags

        // 2. Extract the Tag (e.g., 'a')
        if(hyphenIndex + 1 < Command.length()){
             Indexs[i] = Command.charAt(hyphenIndex + 1);
             privateIndex[i] = Indexs[i]; 
        }

        // 3. Find the Start of the Number
        // Start looking 2 chars after hyphen (skip '-' and tag)
        unsigned int valueStart = hyphenIndex + 2;
        
        // CRITICAL FIX: Skip any spaces between the tag and the number!
        while(valueStart < Command.length() && Command.charAt(valueStart) == ' ') {
            valueStart++;
        }

        // 4. Find the End of the Number (Next space)
        spaceIndex = Command.indexOf(' ', valueStart);
        
        // If no space found, the number goes to the end of the string
        if (spaceIndex == -1) {
            spaceIndex = Command.length(); 
        }

        // 5. Extract and Convert
        // Only try to convert if we actually have characters
        if (valueStart < spaceIndex) {
            String valueStr = Command.substring(valueStart, spaceIndex);
            privateArg[i] = valueStr.toDouble(); 
        }

        // Prepare for next loop
        startIndex = spaceIndex; 
        i++;
    }
}
void serialCommunication::clearArgument(){
    for (int i = 0; i < Max_arguments; i++){
        privateArg[i] = 0.0;
        privateIndex[i] = ' ';
    }
}
void serialCommunication::getArgument(){
    for (int i =0; i < Max_arguments; i++){
        Arguments[i] = privateArg [i];
        Indexs[i] = privateIndex[i];
    }
}
bool serialCommunication::requestK(){
    static unsigned long lastPrint = 0;
    
    // Anti-Spam
    if (millis() - lastPrint > 2000) {
        usbPort.println("Waiting for K... (-a 1 -b 1 -c 1 -d 1)");
        lastPrint = millis();
    }

    // FIX: Use a WHILE loop to drain the buffer instantly
    while (usbPort.available() > 0){
        char incomingChar = usbPort.read();
        
        // Handle Newline (Command finished)
        if (incomingChar == '\n' || incomingChar == '\r'){
            if (incomingCommand.length() > 0) {
                String Command = incomingCommand;
                incomingCommand = ""; // Clear for next time
                Command.trim();
                
                // Debug print
                usbPort.print("Processing: ");
                usbPort.println(Command);
                
                readFrom(0, Command);
                getArgument(); 
                
                // Print Parsed Values
                usbPort.println("K Matrix Updated:");
                for (int i = 0; i < 4; i++){ // Hardcoded to 4 to be safe
                    usbPort.print(i); usbPort.print(": "); 
                    usbPort.println(Arguments[i]);
                }
                return true; // DONE!
            }
        } 
        // Append character if it's not a weird control char
        else if (incomingChar >= 32 && incomingChar <= 126) {
            incomingCommand += incomingChar;
        }
    }   
    return false; // Still waiting
}
//End