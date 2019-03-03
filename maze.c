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
//Maze-kisa
    
void kaanto90(int k, int *suunta);
void pysyviivalla();
void ajayli();
 
void zmain(void)
{
    IR_Start();
    IR_flush();
    
    Ultra_Start();
    int d; 
    
    int k; //kääntymissuunnan muuttuja 0=vasemmalle 1=oikealle
    int koord_x, koord_y;
    int suunta = 1; //0=taakse, 1=eteen, 2=vasemmalle, 3=oikealle
    
    TickType_t start_aika;
    TickType_t stop_aika;
    u_int32_t kierrosaika;
    
    struct sensors_ dig;
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000);
    
    motor_start();
    motor_forward(0,0);
    
    for(;;) //alotusviivalle + stop
    {
        if(SW1_Read() == 0)
        {
            for(;;)
            {
                reflectance_digital(&dig);
                motor_forward(50,200);
            
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
        start_aika = xTaskGetTickCount(); // luo aloitusaika
        print_mqtt("Zumo002/ready", "maze");      
        koord_x = 0;
        koord_y = -1;
        IR_wait();
        print_mqtt("Zumo002/start", "%d", start_aika); //tulosta aloitusaika
        break;
    }   

    // ********************** PÄÄOHJELMA ********************** //
    while(1) 
    {
        reflectance_digital(&dig);
        motor_forward(100,5);
        
        if(dig.l3 == 1 && dig.l2 == 1 && dig.r2 == 1 && dig.r3 == 1) //risteys
        {              
            // koordinaattien päivitys
            if(suunta == 0) // jos suunta taakse
            {
                koord_y--;
            }
            
            if(suunta == 1) // jos suunta eteen
            {
                koord_y++;
            }
            
            if (suunta == 2) // jos suunta vasemmalle
            {
                koord_x--;
            }
            
            if(suunta == 3) // jos suunta oikealle
            {
                koord_x++;
            }
          
            print_mqtt("Zumo002/position","%d %d", koord_x, koord_y);
            
            ajayli(); // ylitä risteys
            
            d = Ultra_GetDistance();
            vTaskDelay(100);
            d = Ultra_GetDistance(); //tuplacheckkaus lukuvirheen varalta
                     
            if(d < 20 && koord_x == 0) // jos este edessä ja keskikaistalla
            {
                k = 0; //käänny vasemmalle
                kaanto90(k, &suunta);
                    
                for(;;) // aja yksi ruutu eteenpäin
                {
                    motor_forward(100,5);
                    pysyviivalla();
                    reflectance_digital(&dig);
                                               
                    if(dig.l3 == 1 && dig.l2 == 1 && dig.r2 == 1 && dig.r3 == 1) // risteys
                    {   
                        // koordinaattien päivitys
                        if(suunta == 0) // jos suunta taakse
                        {
                            koord_y--;
                        }
            
                        if(suunta == 1) // jos suunta eteen
                        {
                            koord_y++;
                        }
            
                        if (suunta == 2) // jos suunta vasemmalle
                        {
                            koord_x--;
                        }
            
                        if(suunta == 3) // jos suunta oikealle
                        {
                            koord_x++;
                        }
          
                        print_mqtt("Zumo002/position","%d %d", koord_x, koord_y);                         
                            
                        ajayli(); //ylitä risteys
                            
                        k = 1; //käänny oikealle
                        kaanto90(k, &suunta);
                        
                        d = Ultra_GetDistance();
                        vTaskDelay(100);
                        d = Ultra_GetDistance(); //tuplacheckkaus lukuvirheen varalta
                            
                        if(d < 20) //jos este edessä
                        {
                            k = 0; //käänny vasemmalle
                            kaanto90(k, &suunta);
                        }
                            
                        break;
                    }                
                }
            }
            
            d = Ultra_GetDistance(); //päivitä välissä ettei tule "haamuestettä" + tuplachechkaus lukuvirheen varalta
            vTaskDelay(100);
            d = Ultra_GetDistance();
            
            if(d < 20 && koord_x != 0) // jos edessä on este ja robotti mazen sivulla
            {                
                if(koord_x < 0) //jos vasemmalla puolella mazea
                {
                    k = 1; // käänny oikealle
                    kaanto90(k, &suunta);
                    
                    for(;;) // aja yksi ruutu eteenpäin
                    {
                        motor_forward(100,5);
                        pysyviivalla();
                        reflectance_digital(&dig);
                                               
                        if(dig.l3 == 1 && dig.l2 == 1 && dig.r2 == 1 && dig.r3 == 1) //risteys
                        {
                            // koordinaattien päivitys
                            if(suunta == 0) // jos suunta taakse
                            {
                                koord_y--;
                            }
            
                            if(suunta == 1) // jos suunta eteen
                            {
                                koord_y++;
                            }
            
                            if (suunta == 2) // jos suunta vasemmalle
                            {
                                koord_x--;
                            }
            
                            if(suunta == 3) // jos suunta oikealle
                            {
                                koord_x++;
                            }
          
                            print_mqtt("Zumo002/position","%d %d", koord_x, koord_y); 
                            
                            ajayli(); // aja risteyksen yli
                            
                            k = 0; //käänny vasemmalle
                            kaanto90(k, &suunta);
                            
                            d = Ultra_GetDistance();
                            vTaskDelay(100);
                            d = Ultra_GetDistance(); //tuplacheckkaus virheiden varalta
                            
                            if(d < 20) //jos edessä este
                            {
                                k = 1; //käänny oikealle
                                kaanto90(k, &suunta);
                            }
                            
                            break;
                        }                
                    }
                }
                
                if(koord_x > 0) // jos oikealla puolella mazea
                {
                    k = 0; //käänny vasemmalle
                    kaanto90(k, &suunta);
                    
                    for(;;) // aja yksi ruutu eteenpäin
                    {
                        motor_forward(100,5);
                        pysyviivalla();
                        reflectance_digital(&dig);
                                               
                        if(dig.l3 == 1 && dig.l2 == 1 && dig.r2 == 1 && dig.r3 == 1) //risteys
                        { 
                            // koordinaattien päivitys
                            if(suunta == 0) // jos suunta taakse
                            {
                                koord_y--;
                            }
            
                            if(suunta == 1) // jos suunta eteen
                            {
                                koord_y++;
                            }
            
                            if (suunta == 2) // jos suunta vasemmalle
                            {
                                koord_x--;
                            }
            
                            if(suunta == 3) // jos suunta oikealle
                            {
                                koord_x++;
                            }
          
                            print_mqtt("Zumo002/position","%d %d", koord_x, koord_y);
                            
                            ajayli(); // aja risteyksen yli
                            
                            k = 1;
                            kaanto90(k, &suunta); //käänny oikealle + tarkasta    
                            
                            d = Ultra_GetDistance();
                            vTaskDelay(100);
                            d = Ultra_GetDistance(); //tuplacheckkaus lukuvirheiden varalta
                            
                            if(d < 20) // jos edessä on este
                            {
                                k = 0; //käänny vasemmalle
                                kaanto90(k, &suunta);
                            }
                            
                            break;
                        }                
                    }   
                }
            }            
            
            if(koord_x == 0) //jos keskikaistalla mazea
            {
                if(suunta == 2) //suunta vasemmalle
                {
                    k = 1;
                    kaanto90(k, &suunta); //kääntö oikealle
                    
                    d = Ultra_GetDistance();
                    vTaskDelay(100);
                    d = Ultra_GetDistance(); //tuplacheckaus
                    
                    if(d < 20) //jos este edessä
                    {
                        k = 0;
                        kaanto90(k, &suunta); // kääntö takaisin vasemmalle
                    }
                }
                
                if(suunta == 3) //suunta oikealle
                {
                    k = 0;
                    kaanto90(k, &suunta); //kääntö vasemmalle
                    
                    d = Ultra_GetDistance();
                    vTaskDelay(100);
                    d = Ultra_GetDistance(); //tuplacheckaus
                    
                    if(d < 20) //jos este edessä
                    {
                        k = 1;
                        kaanto90(k, &suunta); // kääntö takaisin oikealle
                    }
                }
            }
            
            if(koord_x == -2 && suunta == 2) //jos laidassa ja suunta vasemmalle
            {
                k = 1; //käänny oikealle
                kaanto90(k, &suunta);
            }
            
            if(koord_x == 2 && suunta == 3) //jos laidassa ja suunta oikealle
            {
                k = 0; //käänny vasemmalle
                kaanto90(k, &suunta);
            }
            
            if(koord_y == 11 && koord_x < 0 && suunta == 1) //jos loppupäädyssä vasemmalla puolella
            {
                k = 1; //käänny oikealle
                kaanto90(k, &suunta);
            }
            
            if(koord_y == 11 && koord_x > 0 && suunta == 1) //jos loppupäädyssä oikealla puolella
            {
                k = 0; //käänny vasemmalle
                kaanto90(k, &suunta);
            }
            
            if(koord_y == 11 && koord_x == 0) // käänny loppusuoralle
            {
                if(suunta == 2) //jos suunta vasemmalle
                {
                    k = 1; //käänny oikealle
                    kaanto90(k, &suunta);
                }
                
                if(suunta == 3) //jos suunta oikealle
                {
                    k = 0; //käänny vasemmalle
                    kaanto90(k, &suunta);
                }
            }
        }
        
        if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0) //radan lopetus
        {
            motor_stop();
            stop_aika = xTaskGetTickCount(); // hae stop_aika
            kierrosaika = stop_aika - start_aika;
            print_mqtt("Zumo002/stop", "%d", stop_aika);
            print_mqtt("Zumo002/time", "%d", kierrosaika);
            break;
        }
        
        pysyviivalla();
    }
}   

// **************** FUNKTIOT **************** //
void ajayli()
{
    motor_forward(0,0); //pysäytä + aja viivan yli
    motor_forward(100,75);
}

void kaanto90(int k, int *suunta)
{ 
    struct sensors_ dig;
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000);
    
    if(k == 1) //käännös oikealle
    {
        switch(*suunta) //suunta pääohjelmasta
        {   
            case 0:
            {
                *suunta = 2;
                break;
            }
            
            case 1:
            {
                *suunta = 3;
                break;
            }
            
            case 2:
            {
                *suunta = 1;
                break;
            }
            
            case 3:
            {
                *suunta = 0;
                break;
            }
        }
        
        for(int i = 0; i < 42; i++) //90 käännös
        {
            motor_turn(255,0,5);
            motor_backward(0,0);
            motor_turn(0,255,5);
            motor_forward(0,0);    
        }
            
        for(;;) // korjaa suoraksi
        {               
            reflectance_digital(&dig);
            
            if(dig.l3 == 1 || dig.l2 == 1) //jos meni yli
            {
                motor_turn(0,255,5);
                motor_backward(0,0);
                motor_turn(255,0,5);
                motor_forward(0,0);
            }
            
            if(dig.r3 == 1 || dig.r2 == 1) //jos jää vajaaksi
            {
                motor_turn(255,0,5);
                motor_backward(0,0);
                motor_turn(0,255,5);
                motor_forward(0,0);
            }
            
            if(dig.l1 == 1 || dig.r1 == 1)
            {
                break;
            }      
            
            motor_forward(50,5); // estää robotin jäätymisen jos ehdot ei täyty
        }
    }
    
    if(k == 0) // käännös vasemmalle
    {
        switch(*suunta) //suunta pääohjelmasta
        {
            case 0:
            {
                *suunta = 3;
                break;
            }
            
            case 1:
            {
                *suunta = 2;
                break;
            }
            
            case 2:
            {
                *suunta = 0;
                break;
            }
            
            case 3:
            {
                *suunta = 1;
                break;
            }
        }
        
        for(int i = 0; i < 42; i++) // 90 käännös
        {
            motor_turn(0,255,5);
            motor_backward(0,0);
            motor_turn(255,0,5);
            motor_forward(0,0);    
        }
            
        for(;;) // korjaa suoraksi
        {            
            reflectance_digital(&dig);
            
            if(dig.r3 == 1 || dig.r2 == 1) //jos meni yli
            {
                motor_turn(255,0,5);
                motor_backward(0,0);
                motor_turn(0,255,5);
                motor_forward(0,0);
            }
            
            if(dig.l3 == 1 || dig.l2 == 1) //jos jäi vajaaksi
            {
                motor_turn(0,255,5);
                motor_backward(0,0);
                motor_turn(255,0,5);
                motor_forward(0,0);
            }
            
            if(dig.l1 == 1 || dig.r1 == 1)
            {
                break;
            }       
            
            motor_forward(50,5); // estää robotin jäätymisen jos ehdot ei täyty
        }
    }
}

void pysyviivalla()
{
    struct sensors_ dig;
    
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000);
    reflectance_digital(&dig);
    
    if(dig.l3 == 0 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 0 && dig.r2 == 0&& dig.r3 == 0) //pieni korjaus vasemmalle
    {     
        for(;;)
        {
            motor_turn(40,125,5);
            reflectance_digital(&dig);
                   
            if(dig.l3 == 1 && dig.l2 == 1 && dig.r2 == 1 && dig.r3 == 1)// huomaa risteyksen vaikka korjaisi
            {
                break;
            }
            
            if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1)// huomaa kulmaristeyksen vaikka korjaisi
            {
                break;
            }
            
            if(dig.l2 == 0)// korjaus tehty
            {
                break;
            }
        }
    }
        
    if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0) //isompi korjaus vasemmalle
    {        
        for(;;)
        {
            motor_turn(25,135,5);
            reflectance_digital(&dig);
                   
            if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1)// huomaa risteyksen vaikka korjaisi
            {
                break;
            }
            
            if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1)// huomaa kulmaristeyksen vaikka korjaisi
            {
                break;
            }
            
            if(dig.l3 == 0)// korjaus tehty
            {
                break;
            }
        }
    }
        
    if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 0) //pieni korjaus oikealle
    {       
        for(;;)
        {
            motor_turn(125,40,5);
            reflectance_digital(&dig);
                    
            if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1)// huomaa risteyksen vaikka korjaisi
            {
                break;
            }
            
            if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1)// huomaa kulmaristeyksen vaikka korjaisi
            {
                break;
            }
            
            if(dig.r2 == 0)// korjaus tehty
            {
                break;
            }
        }
    }
        
    if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 1 && dig.r3 == 1) //isompi korjaus oikealle
    {       
        for(;;)
        {
            motor_turn(135,25,5);
            reflectance_digital(&dig);
            
            if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1) // huomaa risteyksen vaikka korjaisi
            {
                break;
            }
            
            if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1) // huomaa kulmaristeyksen vaikka korjaisi
            {
                break;
            }
            
            if(dig.r3 == 0)// korjaus tehty
            {
                break;
            }
        }
    }
}

#endif
