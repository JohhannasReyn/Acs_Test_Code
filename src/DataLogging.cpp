#include "DataLogging.hpp"

const int chipSelect = BUILTIN_SDCARD;
File DataFile;


void DataLogSetup(String s){
    
    if (!SD.begin(chipSelect)) {
		Serial.print("Error: Could not create ");
		Serial.print(s);
		Serial.println(".txt, SD card not found.");
        return;
    }

	Serial.print("Created ");
    Serial.print(s);
    Serial.println(".txt on SD card. Ready to be written to.");

    DataFile = SD.open(s.append(".txt").c_str(), FILE_WRITE);

    DataFile.println("---START---");
    
    DataFile.close();
}

void DataLog(double Data[], int size, String s)
{
    DataFile = SD.open(s.append(".txt").c_str(), FILE_WRITE);
    if (DataFile) {
        for(int i = 0 ; i< size;i++) {
            DataFile.print(Data[i]);
            DataFile.print(", ");

            Serial.print(Data[i]);
            Serial.print(", ");
        }

        DataFile.println();
        Serial.println();
        DataFile.close();
    } else {
		Serial.print(s);
		Serial.print(": ");
        for(int i = 0 ; i < size; i++) {
            Serial.print(Data[i]);
            Serial.print(", ");
        }
        Serial.println();
    }
}