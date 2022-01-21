# cfns-hd-so
This code is developed for the Sodaq One in the project Half-Duplex

This application interfaces with
[cfns-half-duplex](https://github.com/PoCDAB/cfns-half-duplex). See the diagram
below.

![Integration with other CFNS systems](integration.png)

## Requirement

### Software
- A program to upload .ino files and .cpp files to a Sodaq One Rev3. An Example is arduino IDE which you can download [here](https://www.arduino.cc/en/software).

### Hardware
- Sodaq One Rev3

## Setup

### Setup LoRaWAN
1. To setup LoRaWAN and the FiPy for the first time see [this link](https://pycom.io/wp-content/uploads/2020/04/Lesson-4-Getting-Connected-with-LoRa.pdf).
2. If it is succesful you know that everything should be able to work. If it is not succesful it can either mean something went wrong or you have no reach. If it's the latter there is nothing you can do except for going closer to a gateway until it does succeed.
3. Open [Half-Duplex_SodaqOne.ino](Half-Duplex_SodaqOne/Half-Duplex_SodaqOne.ino).
4. Replace the placeholders in the method connect_to_TTN with the data of the TTN app you made in step 1.
5. Save [Half-Duplex_SodaqOne.ino](Half-Duplex_SodaqOne/Half-Duplex_SodaqOne.ino).
6. Upload the code to the Sodaq One by pressing _Upload_.
7. You have successfully setup LoRaWAN for the Sodaq One.

# Credit
This repository has been forked from
[SodaqMoja/Sodaq_Ublox_GPS](https://github.com/SodaqMoja/Sodaq_UBlox_GPS).
