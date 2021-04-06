/** 
 *  (c) 2014, Manuel Vonthron - OPAL-RT Technologies, inc.
 */

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <thread>
#include <pthread.h>
#include <math.h>
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#define EC_TIMEOUTMON 500
#define JDOF 1
#define MAX_TORQUE 10
#define ELMO_DOF 33

#define INITIAL_POS 0

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

bool ecat_number_ok = false;
bool ecat_WKC_ok = false;

namespace EtherCAT_Elmo
{
    enum MODE_OF_OPERATION
    {
        ProfilePositionmode = 1,
        ProfileVelocitymode = 3,
        ProfileTorquemode = 4,
        Homingmode = 6,
        InterpolatedPositionmode = 7,
        CyclicSynchronousPositionmode = 8,
        CyclicSynchronousVelocitymode = 9,
        CyclicSynchronousTorquemode = 10,
        CyclicSynchronousTorquewithCommutationAngle = 11
    };

    struct ElmoGoldDevice
    {
        struct elmo_gold_tx
        {
            int32_t targetPosition;
            int32_t targetVelocity;
            int16_t targetTorque;
            uint16_t maxTorque;
            uint16_t controlWord;
            int8_t modeOfOperation;
        };
        struct elmo_gold_rx
        {
            int32_t positionActualValue; // this one !!!!!
            uint32_t hommingSensor;
            uint16_t statusWord;            
            int32_t velocityActualValue;
            int16_t torqueActualValue;
            int32_t positionExternal;
        };
    };
} // namespace EtherCAT_Elmo

EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *rxPDO[JDOF];
EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *txPDO[JDOF];

int ElmoSafteyMode[JDOF];
double control_time_real_;

bool hommingElmo[JDOF];
int stateElmo[JDOF];
int ElmoMode[ELMO_DOF];

double positionElmo, velocityElmo, torqueElmo, positionExternalElmo;
double ELMO_torque[JDOF];

const int FAULT_BIT = 3;
const int OPERATION_ENABLE_BIT = 2;
const int SWITCHED_ON_BIT = 1;
const int READY_TO_SWITCH_ON_BIT = 0;
enum
{
    CW_SHUTDOWN = 6,
    CW_SWITCHON = 7,
    CW_ENABLEOP = 15,
    CW_DISABLEOP = 7,
};


 enum
    {
        EM_POSITION = 11,
        EM_TORQUE = 22,
        EM_DEFAULT = 33,
        EM_COMMUTATION = 44,
    };

/**
 * helper macros
 */
#define READ(idx, sub, buf, comment)    \
    {   \
        buf=0;  \
        int __s = sizeof(buf);    \
        int __ret = ec_SDOread(1, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
        printf("Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment);    \
     }

#define WRITE(idx, sub, buf, value, comment) \
    {   \
        int __s = sizeof(buf);  \
        buf = value;    \
        int __ret = ec_SDOwrite(1, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);  \
        printf("Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", idx, sub, __ret, __s, (unsigned int)buf, comment);    \
    }

#define CHECKERROR()   \
{   \
    ec_readstate();\
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[1].state, ec_slave[1].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[1].ALstatuscode));    \
}


bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord)
{
    if (!(statusWord & (1 << OPERATION_ENABLE_BIT)))
    {
        if (!(statusWord & (1 << SWITCHED_ON_BIT)))
        {
            if (!(statusWord & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if (statusWord & (1 << FAULT_BIT))
                {
                    std::cout << "false1" <<std::endl;
                    controlWord = 0x80;                    
                    return false;
                }
                else
                {
                    std::cout << "false2" <<std::endl;
                    controlWord = CW_SHUTDOWN;                    
                    return false;
                }
            }
            else
            {
                std::cout << "true1" <<std::endl;
                controlWord = CW_SWITCHON;                
                return true;
            }
        }
        else
        {
            std::cout << "true2" <<std::endl;
            controlWord = CW_ENABLEOP;            
            return true;
        }
    }
    else
    {
        std::cout << "true3" <<std::endl;
        controlWord = CW_ENABLEOP;        
        return true;
    }
    std::cout << "false3" <<std::endl;
    controlWord = 0;    
    return false;
}


void simpletest2(char *ifname)
{
     std::cout << "ethercatThread Start" << std::endl;
    char IOmap[4096];
    bool reachedInitial[JDOF] = {false};

    control_time_real_ = 0.0;
    bool exit_middle = false;

    if (ec_init(ifname))
    {
        printf("ELMO : ec_init on %s succeeded.\n", ifname);
    
     /* find and auto-config slaves */
        /* network discovery */
        //ec_config_init()
        if (ec_config_init(FALSE) > 0) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("ELMO : %d slaves found and configured.\n", ec_slavecount); // ec_slavecount -> slave num
            if (ec_slavecount == JDOF)
            {
                ecat_number_ok = true;
            }
            else
            {
                std::cout << "WARNING : SLAVE NUMBER INSUFFICIENT" << std::endl;
            }

            /** CompleteAccess disabled for Elmo driver */
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                //printf("ELMO : Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
                if (!(ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA))
                {
                    printf("ELMO : slave[%d] CA? : false , shutdown request \n ", slave);
                    exit_middle = true;
                }
                ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            if (!exit_middle)
            {
                if (true)
                {
    //                 for (int i=1; i<=ec_slavecount; i++) {

    //                     #define READ(idx, sub, buf, comment)    \
    // {   \
    //     buf=0;  \
    //     int __s = sizeof(buf);    \
    //     int __ret = ec_SDOread(1, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
    //     printf("Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment);    \
    //  }

    //                     int os;
    //                     os = sizeof(map_1c12);
    //                     ec_SDOwrite(slave, 0x1c12, 0, TRUE, os, &map_1c12, EC_TIMEOUTRXM);
    //                     os = sizeof(map_1c13);
    //                     ec_SDOwrite(slave, 0x1c13, 0, TRUE, os, &map_1c13, EC_TIMEOUTRXM);

    //                 int os_buf;
    //                 uint16 map_stauts = {0x1605};
    //                 ec_SDOwrite(i, 0x6040, 0, FALSE, os_buf, "*status word*");

    //                 if(buf16 == 0x218)
    //                 {
    //                     WRITE(i, 0x6040, 0, buf16, 128, "*control word*"); usleep(100000);
    //                     READ(i, 0x6041, 0, buf16, "*status word*");
    //                 }

    //                 WRITE(i, 0x6040, 0, buf16, 0, "*control word*"); usleep(100000);
    //                 READ(i, 0x6041, 0, buf16, "*status word*");

    //                 WRITE(i, 0x6040, 0, buf16, 6, "*control word*"); usleep(100000);
    //                 READ(i, 0x6041, 0, buf16, "*status word*");

    //                 WRITE(i, 0x6040, 0, buf16, 7, "*control word*"); usleep(100000);
    //                 READ(i, 0x6041, 0, buf16, "*status word*");

    //                 WRITE(i, 0x6040, 0, buf16, 15, "*control word*"); usleep(100000);
    //                 READ(i, 0x6041, 0, buf16, "*status word*");

    //                 CHECKERROR(i);
    //                 READ(i, 0x1a0b, 0, buf8, "OpMode Display");

    //                 READ(i, 0x1001, 0, buf8, "Error");
    //             }

    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
                
                    for (int slave = 1; slave <= ec_slavecount; slave++)
                    {
                        //0x1605 :  Target Position             32bit
                        //          Target Velocity             32bit
                        //          Target Torque               16bit
                        //          Max Torque                  16bit
                        //          Control word                16bit
                        //          Modes of Operation          16bit
                        //0x607A : Target Position
                        //uint16 map_1c12[1] = {0x1605};
                        //uint16 map_1c12[1] = {0x1605};
                        //uint16 map_1c12[2] = {0x607A,0x6040};
                       // uint16 map_1c12[2] = {0x6040, 0x6072};
                        uint16 map_1c12[2] = {0x0001, 0x1605};

                        //0x1a00 :  position actual value       32bit
                        //          Digital Inputs              32bit
                        //          Status word                 16bit
                        //0x6064 :  position actual value       32bit
                        //0x6041 :  Status word                 16bit
                        //0x60FD :  Digital Inputs              32bit
                        //0x1a11 :  velocity actual value       32bit
                        //0x606c :  velocity actual value       32bit
                        //0x1a12 :  Torque demand Value         16bit
                        //0x1a13 :  Torque actual value         16bit
                        //0x6077 :  Torque actual value         16bit
                        //0x1a1e :  Auxiliary position value    32bit
                        uint16 map_1c13[5] = {0x0004, 0x1a00, 0x1a11, 0x1a13, 0x1a1e}; //, 0x1a12};
                        //uint16 map_1c13[4] = {0x1a00, 0x1a11, 0x1a13, 0x1a1e}; //, 0x1a12};
                        //uint16 map_1c13[4] = {0x6041, 0x6064, 0x6077, 0x606C}; //, 0x1a12}; homming X
                        //uint16 map_1c13[3] = {0x1a00 , 0x606C, 0x6077};
                        //uint16 map_1c13[2] = {0x6041,0x6064};
                        //uint16 map_1c13[3] = {0x0002, 0x1a11, 0x1a00};
                        //uint16 map_1c13[6] = {0x0005, 0x1a04, 0x1a11, 0x1a12, 0x1a1e, 0X1a1c};
                        
                        int os;
                        os = sizeof(map_1c12);                        
                        int retVal = 0;
                        retVal = ec_SDOwrite(slave, 0x1c12, 0, TRUE, os, &map_1c12, EC_TIMEOUTRXM);
                        std::cout << "retVal1 : " << retVal <<std::endl;
                        os = sizeof(map_1c13);                        
                        retVal = ec_SDOwrite(slave, 0x1c13, 0, TRUE, os, &map_1c13, EC_TIMEOUTRXM);                        
                        std::cout << "retVal2 : " << retVal <<std::endl;
                        
                    }

                    /** if CA disable => automapping works */
                    ec_config_map(&IOmap);

                    /* wait for all slaves to reach SAFE_OP state */
                    printf("ELMO : EC WAITING STATE TO SAFE_OP\n");
                    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

                    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

                    printf("ELMO : Request operational state for all slaves. Calculated workcounter : %d\n", expectedWKC);
                    if (expectedWKC != 3 * JDOF)
                    {
                        std::cout << "WARNING : Calculated Workcounter insufficient!" << std::endl;
                        ecat_WKC_ok = true;
                    }
                    /** going operational */
                    ec_slave[0].state = EC_STATE_OPERATIONAL;

                    /* send one valid process data to make outputs in slaves happy*/

                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);


                    /* request OP state for all slaves */
                    ec_writestate(0);

                    /* wait for all slaves to reach OP state */
                    int wait_cnt = 40;
                    do
                    {
                        ec_send_processdata();
                        ec_receive_processdata(EC_TIMEOUTRET);
                        ec_statecheck(0, EC_STATE_OPERATIONAL, 5000);
                    } while (wait_cnt-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

                    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
                    {
                        if (ecat_number_ok && ecat_WKC_ok)
                        {
                            std::cout << "All slaves Status OK" << std::endl;
                        }
                        else
                        {
                            std::cout << "Please Check Slave status" << std::endl;
                        }
                        std::cout << "STARTING IN 3 ... " << std::endl;
                        printf("ELMO : Operational state reached for all slaves! Starting in ... 3... ");
                        fflush(stdout);
                        usleep(1000000);//std::this_thread::sleep_for(std::chrono::seconds(1));
                        printf("2... ");
                        fflush(stdout);
                        usleep(1000000);//std::this_thread::sleep_for(std::chrono::seconds(1));
                        printf("1... ");
                        fflush(stdout);
                        usleep(1000000);//std::this_thread::sleep_for(std::chrono::seconds(1));
                        printf("0... Start! \n");

                        inOP = TRUE;

                         /* cyclic loop */
                         
                        for (int slave = 1; slave <= ec_slavecount; slave++)
                        {
                            txPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slave].outputs);
                            rxPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slave].inputs);
                        }
                        

                        std::chrono::duration<double> time_from_begin;
                        std::chrono::duration<double> time_err_reset = std::chrono::seconds(0);

                        int cycle_time = 1000; // 1ms, 1kHz
                        int cycle_count = 0; 
                        std::chrono::microseconds cycletime(cycle_time);
                        

                        std::chrono::steady_clock::time_point tp[10];
                        std::chrono::steady_clock::time_point time_until;

                        std::chrono::duration<double> td[8];
                        double to_ratio, to_calib;

                        double pwait_time = 1.0;
                        int c_count = 0;

                        double d_min = 1000;
                        double d_max = 0;
                        double d_mean = 0;

                        double d1_min = 1000;
                        double d1_max = 0;
                        double d1_mean = 0;

                        int oc_cnt = 0;
                        int oct_cnt = 0;
                        int err_cnt = 0;

                        positionElmo = 0.0;
                        velocityElmo = 0.0;
                        torqueElmo = 0.0;
                        positionExternalElmo = 0.0;

                        std::chrono::steady_clock::time_point st_start_time;
                        st_start_time = std::chrono::steady_clock::now();

                        for (int i = 0; i < ec_slavecount; i++)
                        {
                            ElmoSafteyMode[i] = 0;
                        }

                        while(true)
                        {
                            //Ethercat Loop begins :: RT Thread
                            tp[0] = std::chrono::steady_clock::now();

                            std::this_thread::sleep_until(st_start_time + cycle_count * cycletime);

                            tp[1] = std::chrono::steady_clock::now();
                            time_from_begin = std::chrono::steady_clock::now() - st_start_time;
                            control_time_real_ = time_from_begin.count();

                            /** PDO I/O refresh */
                            ec_send_processdata();

                            tp[2] = std::chrono::steady_clock::now();
                            wkc = ec_receive_processdata(0);

                            tp[3] = std::chrono::steady_clock::now();

                            td[0] = st_start_time + cycle_count * cycletime - tp[0];
                            td[1] = tp[1] - tp[0];

                            td[2] = tp[2] - tp[1];
                            td[3] = tp[3] - tp[2];

                            if (td[3] > std::chrono::microseconds(250))
                            {
                                oc_cnt++;
                            }

                            if (tp[3] > st_start_time + (cycle_count + 1) * cycletime)
                            {
                                std::cout << "## ELMO LOOP INSTABILITY DETECTED ##\n START TIME DELAY :" << -td[0].count() * 1E+6 << " us\n THREAD SYNC TIME : " << td[1].count() * 1E+6 << " us\n RECV ELMO TIME : " << td[3].count() * 1E+6 << " us \t last tick : " << td[5].count() * 1E+6 << " us \t last send process : " << td[6].count() * 1E+6 << " us\n LAST LOOP :" << td[4].count() * 1E+6 << " us\n " << std::endl;
                                std::cout << "If you see this warning message often, Rebooting robot is recommanded" << std::endl;
                                
                                err_cnt++;
                                //std::cout << err_cnt << "   " << cycle_count <<std::endl;
                                if(err_cnt > 10)
                                {                                    
                                    break;
                                }
                            }
                            td[5] = td[3];
                               // std::cout << "WKC" << wkc <<std::endl;
                            if (wkc >= expectedWKC)
                            {
                                for (int slave = 1; slave <= ec_slavecount; slave++)
                                {
                                    //std::cout << controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord) <<std::endl;                                    

                                    if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                                    //if (controlWordGenerate(val->status,target->status))
                                    {
                                        //std::cout << "Test" <<std::endl;
                                        reachedInitial[slave - 1] = true;    
                                    }
                                }

                                for (int slave = 1; slave <= ec_slavecount; slave++)
                                {
                                    //std::cout << "Test2" <<std::endl;

                                    if (reachedInitial[slave - 1])
                                    {          
                                        txPDO[slave-1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousVelocitymode;
                                        txPDO[slave-1]->targetVelocity = (int16) 0;
                                  

                                        // std::cout << "Test3" <<std::endl;
                                        // std::cout << target->status <<std::endl;
                                        // //std::cout << val->torque <<std::endl;
                                        // std::cout << val->status <<std::endl;                   
                                        // std::cout << val->position <<std::endl;                   
                                        std::cout << txPDO[slave - 1]->controlWord <<std::endl;
                                        //std::cout << rxPDO[slave - 1]->velocityActualValue <<std::endl;
                                        std::cout << rxPDO[slave - 1]->statusWord <<std::endl;
                                        std::cout << rxPDO[slave - 1]->velocityActualValue <<std::endl;
                                        std::cout << rxPDO[slave - 1]->hommingSensor <<std::endl;
                                        
                                        //Get status
                                        //positionElmo = rxPDO[slave - 1]->velocityActualValue;
                                        //txPDO[0]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousVelocitymode;
                                       

                                      /*
                                        std::cout << "torque"<< torqueElmo <<std::endl;
                                         
                                        hommingElmo[slave - 1] =
                                            (((uint32_t)ec_slave[slave].inputs[4]) +
                                             ((uint32_t)ec_slave[slave].inputs[5] << 8) +
                                             ((uint32_t)ec_slave[slave].inputs[6] << 16) +
                                             ((uint32_t)ec_slave[slave].inputs[7] << 24));
                                        
                                        stateElmo[slave - 1] =
                                            (((uint16_t)ec_slave[slave].inputs[8]) +
                                             ((uint16_t)ec_slave[slave].inputs[9] << 8));

                                        velocityElmo =
                                            (((int32_t)ec_slave[slave].inputs[10]) +
                                             ((int32_t)ec_slave[slave].inputs[11] << 8) +
                                             ((int32_t)ec_slave[slave].inputs[12] << 16) +
                                             ((int32_t)ec_slave[slave].inputs[13] << 24));
                                             */
                                        /*
                                         //ElmoMode[slave - 1] = EM_TORQUE;
                                         ELMO_torque[slave - 1] = 1;
                                         txPDO[0]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                         txPDO[0]->targetTorque = (int) 1 ;
                                         torqueElmo =rxPDO[slave - 1]->torqueActualValue;
                                         
                                         torqueElmo=
                                            (((int16_t)ec_slave[slave].inputs[14]) +
                                             ((int16_t)ec_slave[slave].inputs[15] << 8));
                                                                          
                                        
                                        positionExternalElmo =
                                            (((int32_t)ec_slave[slave].inputs[16]) +
                                             ((int32_t)ec_slave[slave].inputs[17] << 8) +
                                             ((int32_t)ec_slave[slave].inputs[18] << 16) +
                                             ((int32_t)ec_slave[slave].inputs[19] << 24)); //???
                                        
                                        txPDO[slave - 1]->maxTorque = (uint16)MAX_TORQUE; // originaly 1000                                        
                                       */
                                    }
                                }                                 
                            }
                            else{
                                break;
                            }     


                        cycle_count++;





                            //if(reachedInitial[slave - 1] == true) 
                            //{
                            //    break;
                            //}
                        }
                        
                    }

                }
            }
        }
    }
}


void *ecatcheck( void *ptr )
{
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf(".");
        }
        usleep(250);
    }
}




int main(int argc, char *argv[])
{
    int iret1;
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
      //pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      iret1 = pthread_create( &thread1, NULL, &ecatcheck, (void*) &ctime); // (void) &ctime
      //osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
      /* start cyclic part */
      simpletest2(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }
   printf("End program\n");
   return (0);
}


