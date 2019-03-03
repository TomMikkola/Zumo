#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#define PI 3.14159265
#include "I2C_Common.h"
#include "task.h"
#include "Accel_magnet.h"

#if 1
//Sumo-zumo
    
void tornado();
double laskekulma(double x, double y);   
void lue_osuma();
    
void zmain(void)
{    
    motor_start();
    motor_forward(0,0);
    
    TickType_t start_aika;
    TickType_t stop_aika;
    u_int32_t ajo_aika;
    
    struct sensors_ dig;
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 9000, 9000, 9000, 9000);
    
    LSM303D_Start();
    
    Ultra_Start();
    int d = Ultra_GetDistance();
    
    IR_Start();
    IR_flush();
         
    for(;;) //alotusviivalle + stop
    {
        if(SW1_Read() == 0)
        {
            for(;;)
            {
                reflectance_digital(&dig);
                motor_forward(50,100);
            
                if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1)
                {
                    motor_forward(0,0);
                    break;
                }
            }        
            break;
        }
    }   
    
    for(;;) // odota kaukosäädintä + start + ajanotto
    {
        start_aika = xTaskGetTickCount(); // hae aloitusaika
        print_mqtt("Zumo002/ready", "zumo");      
        IR_wait();
        print_mqtt("Zumo002/start", "%d", start_aika);
        break; 
    }   
         
    // **** PÄÄOHJELMA **** //
    motor_turn(55,25,100); // pieni alkukiihdytys, ettei lue törmäykseksi
    motor_turn(155,125,1150); // aja keskelle
        
    for(;;)
    {
        tornado();
        d = Ultra_GetDistance();
        lue_osuma();
             
        if(d < 25) //jos havaitsee toisen robotin
        {                          
            for(;;)
            {
                reflectance_digital(&dig);
                lue_osuma();
                                                                
                if(dig.l3 == 1 || dig.l2 == 1 || dig.l1 == 1 || dig.r1 == 1 || dig.r2 == 1 || dig.r3 == 1) //jos ulkokehä tulee vastaan tai keula nousee ilmaan
                {
                    motor_forward(0,0);
                    vTaskDelay(100);
                    
                    reflectance_digital(&dig); //tupla checkkaus pelkän "keulahypyn" varalta 
                    
                    if(dig.l3 == 1 || dig.l2 == 1 || dig.l1 == 1 || dig.r1 == 1 || dig.r2 == 1 || dig.r3 == 1)
                    {
                        motor_backward(0,0);
                        motor_backward(50,50); // pieni alkukiihdytys, ettei lue osumaksi
                        motor_backward(150,450);
                        
                        for(int i = 0; i < 60; i++) //käännös
                        {
                            motor_forward(0,0);
                            motor_turn(255,0,5);
                            motor_backward(0,0);
                            motor_turn(0,255,5);
                            motor_forward(0,0);    
                        }
                        break;
                    }                        
                    break;
                }  
                
                motor_turn(155,125,10); // pieni alkukiihdytys
                motor_turn(255,225,10); // aja kohti
            }
        }
        
        reflectance_digital(&dig);
        if(dig.l3 == 1 || dig.l2 == 1 || dig.l1 == 1 || dig.r1 == 1 || dig.r2 == 1 || dig.r3 == 1) //jos ulkokehä tulee vastaan tai keula nousee ilmaan
        {
            motor_forward(0,0); // pysähdy + odota
            vTaskDelay(100);
                    
            reflectance_digital(&dig); //tupla checkkaus pelkän "keulahypyn" varalta               
            if(dig.l3 == 1 || dig.l2 == 1 || dig.l1 == 1 || dig.r1 == 1 || dig.r2 == 1 || dig.r3 == 1)
            {
                motor_backward(0,0);
                motor_backward(50,50); // pieni alkukiihdytys ettei lue osumaksi
                motor_backward(150,450);
                
                for(int i = 0; i < 60; i++) //käännös
                {
                    motor_forward(0,0);
                    motor_turn(255,0,5);
                    motor_backward(0,0);
                    motor_turn(0,255,5);
                    motor_forward(0,0);    
                }
            }    
        }
        
        if(SW1_Read() == 0) // user buttonista ajantulostus
        {
            motor_stop();
            stop_aika = xTaskGetTickCount();
            print_mqtt("Zumo002/stop","%d", stop_aika);
            ajo_aika = stop_aika - start_aika;
            print_mqtt("Zumo002/time","%d", ajo_aika);
            break;
        }
    }
}   

void tornado() //pyöri paikallaan
{
    motor_turn(255,0,10);
    motor_backward(0,0);
    motor_turn(0,255,10);
    motor_forward(0,0);
}

double laskekulma(double x,double y) //laske osumakulma
{
    double kulma;
    
    kulma = atan2(y,-x) * 180 / PI;
    
    if(kulma < 0)
    {
        kulma += 360;
    }
    
    return kulma;
}

void lue_osuma() //määrittää raja-arvot kiihtyvyysanturille ja tulostaa osumakulman
{
    struct accData_ data;
    
    LSM303D_Read_Acc(&data);
    
    double x = data.accX;
    double y = data.accY;
    double kulma;
    u_int32_t osuma_aika;
    
    if(data.accX < -10800) //osuma edestä
    {
        printf("%.2f %.2f\n",x, y);
        kulma = laskekulma(x, y);
        osuma_aika = xTaskGetTickCount();
        print_mqtt("Zumo002/hit","%d %.0f",osuma_aika, kulma);
        vTaskDelay(400);
    }
        
    if(data.accX > 13200) //osuma takaata
    {
        printf("%.2f %.2f\n",x, y);
        kulma = laskekulma(x, y);
        osuma_aika = xTaskGetTickCount();
        print_mqtt("Zumo002/hit","%d %.0f",osuma_aika, kulma);
        vTaskDelay(400);
    }
        
    if(data.accY < -12800) //osuma vasemmalta
    {
        printf("%.2f %.2f\n",x, y);
        kulma = laskekulma(x, y);
        osuma_aika = xTaskGetTickCount();
        print_mqtt("Zumo002/hit","%d %.0f",osuma_aika, kulma);
        vTaskDelay(400);
    }
        
    if(data.accY > 13200) //osuma oikealta
    {
        printf("%.2f %.2f\n",x, y);
        kulma = laskekulma(x, y);
        osuma_aika = xTaskGetTickCount();
        print_mqtt("Zumo002/hit","%d %.0f",osuma_aika, kulma);
        vTaskDelay(400);
    }
}
#endif
