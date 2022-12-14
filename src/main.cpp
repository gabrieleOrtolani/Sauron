//
// Created by alfyhack on 09/11/19.
//

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <uartSubSystem.h>
#include <dataTransfert_AllInclude.h>
#include <ConcreteObserver.h>

// #include <ctrlSource.h>

// #include "../../qtScorbotUi/UI_useCase/MotorCTRL.h"

using namespace std;
using namespace DataPrimitive;
using namespace DataManipolation;
// using namespace CTRL;
ScorInterface *scorbot;
ComUartAdapter *uartCom;
/*
QPushButton *mn[nMot];
QPushButton *mp[nMot];
QPushButton *roll[2];
QPushButton *pitch[2];
QSignalMapper *signalMapper;


enum MotState
{
    controllMode,
    freeMode,
    stopMode
};
MotState motState = freeMode;
*/
void ctrlThMain(int);

void set_realtime_priority()
{
    int ret;

    // We'll operate on the currently running thread.
    pthread_t this_thread = pthread_self();
    // struct sched_param is used to store the scheduling priority
    struct sched_param params;

    // We'll set the priority to the maximum.
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    std::cout << "Trying to set thread realtime prio = " << params.sched_priority << std::endl;

    // Attempt to set thread real-time priority to the SCHED_FIFO policy
    ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if (ret != 0)
    {
        // Print the error
        std::cout << "Unsuccessful in setting thread realtime prio" << std::endl;
        return;
    } // Now verify the change in thread priority
    int policy = 0;
    ret = pthread_getschedparam(this_thread, &policy, &params);
    if (ret != 0)
    {
        std::cout << "Couldn't retrieve real-time scheduling paramers" << std::endl;
        return;
    }

    // Check the correct policy was applied
    if (policy != SCHED_FIFO)
    {
        std::cout << "Scheduling is NOT SCHED_FIFO!" << std::endl;
    }
    else
    {
        std::cout << "SCHED_FIFO OK" << std::endl;
    }

    // Print thread scheduling priority
    std::cout << "Thread priority is " << params.sched_priority << std::endl;
}

ConcreteObserver *obs;

int main(int argc, char *argv[])
{
    /// Initialize board

    try
    {
        //        scorbot = &AdapterFactory::makeUart("/dev/ttyUSB0", B115200);
        uartCom = &ComUartAdapter::getInstance("/dev/ttyACM0");
        uartCom->changeDeviceVel(B115200);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        exit(-1);
    }
    scorbot = uartCom;
    obs = new ConcreteObserver(scorbot);

    /// Setting test
    settingsBoard pData;
    SettingBoard_C *p = new SettingBoard_C(pData);

    // Setting per ora di default
    // curr neg => no limit
    p->setMotorLimit(Mot1, 2160, 24300, -19450);
    p->setMotorLimit(Mot2, 2960, 16200, -1000);
    p->setMotorLimit(Mot3, 2960, 21900, -7300);
    p->setMotorLimit(Mot4, 2160, 10000, -10000);
    p->setMotorLimit(Mot5, 2400, 10000, -10000);
    p->setMotorLimit(Mot6, 2160, 5770, -10);
    p->setAdcDiff(false);
    p->setAdcRef(in1V1);
    p->setPWMfreq(hz4k);
    SettingBoard_C *pRecive;
    std::cout << "============================================================\n";
    std::cout << "Default Settings:\n";
    p->printSetting();
    std::cout << "\n";

    /// Test notify SettingBoard
    obs->enableSettingBoard(true);
    std::cout << "============================================================\n";
    std::cout << "Invio Richiesta alla scheda di una setting Board\n";
    pRecive = scorbot->getSetting_board();
    std::cout << "Aspetto 2 secondi e mi aspetto che venga printata la notify di ConcreteObserver\n";
    sleep(2); // Mi aspetto che ConcreteObserver printi un pacchetto ricevuto!!!
    std::cout << "\n\n============================================================\n";
    std::cout << "Ecco cosa ho ricevuto da interfaccia diretta\n";
    pRecive->printSetting();
    delete pRecive;
    obs->enableSettingBoard(false);
    std::cout << "============================================================\n";
    std::cout << "\n";
    std::cout << "Settings read from local:\n";
    pRecive = scorbot->getSetting_local();
    pRecive->printSetting();
    delete pRecive;
    std::cout << "\n";
    std::cout << "Sending default settings:\n";
    scorbot->setSetting(*p);
    std::cout << "\n";
    std::cout << "Settings read from local:\n";
    pRecive = scorbot->getSetting_local();
    pRecive->printSetting();
    delete pRecive;
    std::cout << "\n";
    std::cout << "Settings from board directly:\n";
    pRecive = scorbot->getSetting_board();
    pRecive->printSetting();
    delete pRecive;
    std::cout << "============================================================\n";

    /// Data read test
    EncoderMot *e;
    CurrentMot *c;
    AllSensor *allS;
    struct timespec lastPackT, newPackT, deltaPackT;

    /// Encoder test
    obs->enableEncoderData(true);
    std::cout << "\n\n============================================================\n";
    std::cout << "Test della lettura degli encoder\n";
    sleep(1);
    scorbot->setSampleTimeCur(50000 * 1000UL); // no send
    scorbot->setSampleTimeEn(50 * 1000UL);
    clock_gettime(CLOCK_MONOTONIC_RAW, &lastPackT);
    timerclearSpec(&deltaPackT);
    for (int i = 0; i < 1; i++)
    {
        e = obs->getValidEncoderWait(&newPackT);
        timersubSpec(&newPackT, &lastPackT, &deltaPackT);
        timeStampSpec(&deltaPackT, "deltaPackT");
        if (deltaPackT.tv_nsec < (500 * 1000UL))
            std::cout << "too fast\n"; // falsi positivi dovuti a i ritardi dei thread
        lastPackT = newPackT;
        std::cout << "\n\t##Recived:\n";
        e->printEncoder();
        if (i == 100)
        {
            scorbot->setSampleTimeEn(1 * 1000UL); // max speed
        }
        delete e;
        std::cout << "count: ";
        std::cout << i << "\n";
        std::cout << "\n";
    }
    obs->enableEncoderData(false);

    /*	ABBIAMO TOLTO QUESTA PARTE PERCHE DAVA PROBLEMI ALLA CONNESSIONE
        /// Current test
        obs->enableCurrentData(true);
        std::cout << "\n\n============================================================\n";
        std::cout << "Test della lettura delle correnti \n";
        sleep(1);
        //scorbot->setSampleTimeEn(50000 * 1000UL);  // no send
        //scorbot->setSampleTimeCur(50 * 1000UL);
        //clock_gettime(CLOCK_MONOTONIC_RAW, &lastPackT);
        //timerclearSpec(&deltaPackT);
        for (int i = 0; i < 100; i++){
            c = obs->getValidCurrentWait(&newPackT);
            timersubSpec(&newPackT, &lastPackT, &deltaPackT);
            timeStampSpec(&deltaPackT, "deltaPackT");
            lastPackT = newPackT;
            std::cout << "\n\t##Recived:\n";
            c->printCurrent();
            if (deltaPackT.tv_nsec < (500 * 1000UL))
                std::cout << "too fast\n";
            if (i == 100){
                scorbot->setSampleTimeCur(3 * 1000UL); //max speed
            }
            delete c;
            std::cout << "count: ";
            std::cout << i << "\n";
            std::cout << "\n";
        }
        std::cout << "\n\n============================================================\n";
        std::cout << "End tests\n";
        obs->enableCurrentData(false);
    */

    scorbot->goHome();
    sleep(15);

    while (1)
    {
        //sleep(15);
        //std::cout << "QUESTA È LA GO HOME NEL WHILE\n";
        //scorbot->goHome();
        static int mySpeed = 10;
        
        ctrlThMain(mySpeed);
        mySpeed+=20;
        std::cout << "QUESTA È LA MOSSA\n";
        std::cout << speed;
    }

    return 0;
}

void ctrlThMain(int speed)
{

    /// Variabili del Controllore
    EncoderMot *enFeed; // Puntatore per i dati ricevuti
    EncoderMot *refPid = new EncoderMot();
    SpeedMot *pwm = new SpeedMot();

    while (!scorbot->isConnect()){
        sleep(1);
        std::cout << "CONNECTING...\n";
    }

    short sign;
    //static int speed = 0;
    int i =0;


        //speed = 0; // m->win->freeMovWin->ui->pwmSpeed->value();
        for (int i = Mot6; i < nMot; i++)
        { 
            std::cout << i << std::endl;
            
            
            sign = -1;
            pwm->pwmSet((motCode)i, sign * speed);
            scorbot->sendVel(*pwm);
            sleep(3);
            
            
            sign = 1;
            pwm->pwmSet((motCode)i, sign * speed);
            scorbot->sendVel(*pwm);
            sleep(3);
            

            speed+=10;

            pwm->pwmSet((motCode)i, 0);
        }
}