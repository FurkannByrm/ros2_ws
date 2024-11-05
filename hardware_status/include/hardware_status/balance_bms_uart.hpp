#ifndef BALANCE_BMS_UART_HPP
#define BALANCE_BMS_UART_HPP

#include <string>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <errno.h>
#include <stdio.h>

constexpr auto xfer_buffer_length = 13;
constexpr auto min_number_cells = 1;
constexpr auto max_number_cells = 48;
constexpr auto min_number_temp_sensors = 1;
constexpr auto max_number_temp_sensors = 16;


class Balance_BMS_UART
{

public:
    
        enum COMMAND{

            VOUT_IOUT_SOC = 0x90,
            MIN_MAX_CELL_VOLTAGE = 0x91,
            MIN_MAX_TEMPERATURE = 0x92,
            DISCHARGE_CHARGE_MOS_STATUS = 0x93,
            STATUS_INFO = 0x94,
            CELL_VOLTAGES = 0x95,
            CELL_TEMPERATURE = 0x96,
            CELL_BALANCE_STATE = 0x97,
            FAILURE_CODES = 0x98,
            DISCHRG_FET = 0xD9,
            CHRG_FET = 0xDA,
            BMS_RESET = 0x00,
            
        };

/*
*@FurkannByrm get struct holds all the data collected froö the BMS and is populated using the update() API
*/

     struct 
     {
        //data from 0x90
        double packVoltage; // pressure
        double packCurrent; // acquisition
        double packSOC;     // state of charge

        //data from 0x91
        double maxCellmV; // maximum monomer voltage (mv)
        int maxCellVNum;  // maximum unit voltage cell No. 
        double minCellmV; // minimum monomer voltage (mV)
        int minCellVNum;  // minimum unit voltage cell No.
        double cellDiff;  // difference betweem cells

        //date from 0x92 
        int tempMax;        //maximum monomer temperature(40 offset,°C)
        int tempMin;        //maximum monomer temperature cell No.
        double tempAverage; //Average Temperature

        //data from 0x93
        std::string chargeDischargeStatus; // charge/discharge status (0 stationary, 1 charge , 2 discharge)
        bool chargeFetState;
        bool dischargeFetState; 
        int bmsHeartBeat;
        int resCapacitymAh;

        //data from 0x94
        int numberOfCells;     //amout of cells
        int numOfTempSensors;  //amout of temp sensors
        bool chargeState;      //charger status 0=disconnected 1=connected
        bool loadState;        // load status 0=disconnected 1=connected
        bool dIO[8];           // no info
        int bmsCycles;         //charge / discharge cycles

        //data from 0x95
        double cellVmV[48];    //store cell voltages in mV
        
        //data from 0x96
        int cellTemperature[16]; // array of cell Temperature sensors

        //data from 0x97
        bool cellBalanceState[48]; // bool array of cell balance states
        bool cellBalanceActivate; // bool is cell balance active

        //debug data string
        std::string aDebug;
        
     }get;

/*
*@FurkannByrm alarm struct holds booleans corresponding to possible alarms
*the BMS can report
*/

struct
{
    // data from 0x98
        /* 0x00 */
        bool levelOneCellVoltageTooHigh;
        bool levelTwoCellVoltageTooHigh;
        bool levelOneCellVoltageTooLow;
        bool levelTwoCellVoltageTooLow;
        bool levelOnePackVoltageTooHigh;
        bool levelTwoPackVoltageTooHigh;
        bool levelOnePackVoltageTooLow;
        bool levelTwoPackVoltageTooLow;

        /* 0x01 */
        bool levelOneChargeTempTooHigh;
        bool levelTwoChargeTempTooHigh;
        bool levelOneChargeTempTooLow;
        bool levelTwoChargeTempTooLow;
        bool levelOneDischargeTempTooHigh;
        bool levelTwoDischargeTempTooHigh;
        bool levelOneDischargeTempTooLow;
        bool levelTwoDischargeTempTooLow;

        /* 0x02 */
        bool levelOneChargeCurrentTooHigh;
        bool levelTwoChargeCurrentTooHigh;
        bool levelOneDischargeCurrentTooHigh;
        bool levelTwoDischargeCurrentTooHigh;
        bool levelOneStateOfChargeTooHigh;
        bool levelTwoStateOfChargeTooHigh;
        bool levelOneStateOfChargeTooLow;
        bool levelTwoStateOfChargeTooLow;

        /* 0x03 */
        bool levelOneCellVoltageDifferenceTooHigh;
        bool levelTwoCellVoltageDifferenceTooHigh;
        bool levelOneTempSensorDifferenceTooHigh;
        bool levelTwoTempSensorDifferenceTooHigh;

        /* 0x04 */
        bool chargeFETTemperatureTooHigh;
        bool dischargeFETTemperatureTooHigh;
        bool failureOfChargeFETTemperatureSensor;
        bool failureOfDischargeFETTemperatureSensor;
        bool failureOfChargeFETAdhesion;
        bool failureOfDischargeFETAdhesion;
        bool failureOfChargeFETTBreaker;
        bool failureOfDischargeFETBreaker;

        /* 0x05 */
        bool failureOfAFEAcquisitionModule;
        bool failureOfVoltageSensorModule;
        bool failureOfTemperatureSensorModule;
        bool failureOfEEPROMStorageModule;
        bool failureOfRealtimeClockModule;
        bool failureOfPrechargeModule;
        bool failureOfVehicleCommunicationModule;
        bool failureOfIntranetCommunicationModule;

        /* 0x06 */
        bool failureOfCurrentSensorModule;
        bool failureOfMainVoltageSensorModule;
        bool failureOfShortCircuitProtection;
        bool failureOfLowVoltageNoCharging;
}alarm;


    Balance_BMS_UART(const std::string& serialDev);

    bool Init();
    bool update();
    bool getPackMeasurements();
    bool getPackTemp();
    bool getMinMaxCellVoltage();
    bool getStatusInfo();
    bool getCellVoltages();
    bool getCellTemperature();
    bool getCellBalanceState();
    bool getFailureCodes();
    bool setDischargeMOS(bool sw);
    bool setChargeMOS(bool sw);
    bool getDischargeChargeMosStatus();
    bool setBmsReset();

private:
    
    bool readyRead(bool delayed = false);
    void sendCommand(COMMAND cmdID);
    bool receiveBytes(void);
    bool validateChecksum();
    void barfRXBuffer();
    int my_serialIntf;

  
    uint8_t my_txBuffer[xfer_buffer_length];

  
    uint8_t my_rxBuffer[xfer_buffer_length];
    
    
    fd_set readfd;

};








#endif // BALANCE_BMS_UART_HPP