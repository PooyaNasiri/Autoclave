# include <mega64.h>
# include <delay.h>
# include <stdio.h>
# include <alcd.h>
# include <math.h>
# include <i2c.h>
# include <ds1307.h>
# include <spi.h>
# include <stdbool.h>

#define ver   1.12

#define back  !PIND.5 
#define set   !PINC.1           
#define down  !PINC.3 
#define right !PINC.5 
#define left  !PINC.7
#define up    !PINA.7
#define start !PINE.2 
#define mdo   !PINE.3 

#define wp    PORTC.0 
#define vv    PORTC.2 
#define out   PORTC.4 
#define wv    PORTC.6   

#define dl_close1  (PORTG=(PORTG|0x01))
#define dl_close2  (PORTD.2=0)
#define dl_open1   (PORTG=(PORTG&0xfe))
#define dl_open2   (PORTD.2=1) 
#define dl_off1    (PORTG=(PORTG&0xfe))
#define dl_off2    (PORTD.2=0)
#define buzz_on    (PORTG=(PORTG|0x02)) 
#define buzz_off   (PORTG=(PORTG&0xfd))  
#define bal_on     (PORTG=(PORTG|0x04))
#define bal_off    (PORTG=(PORTG&0xfb)) 

#define spare     PORTD.3  

#define el        PORTD.4 
#define vp        PORTD.6  
#define Err_LED   PORTD.7

#define temp_avg_count    60
#define adc_ref_Volt      2.957
#define ADC_VREF_TYPE     ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR))
#define SET_SPID_SS_LOW   PORTB.0=0;
#define SET_SPID_SS_HIGH  PORTB.0=1;

eeprom char _mode = 0;
eeprom float temp_off = 0, press_off = 0;
eeprom unsigned int complete = 0;
eeprom bool loaded = false;
char* mode[4] = { "Normal", "Fast", "B&D", "121" },
     * status[16] = { "Ready", "Steam out", "Balancing", "PreHeat" , "PreVac1", "PreVac2", "PreVac3", "", "Steam in",
                     "Sterilize", "", "Steam out", "PostVac1", "PostVac2", "PostVac3", "Complete" },
     string[21], year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0, yy = 0, _mm = 0, dd = 0, hh = 0, mm = 0, phase = 0, prePhase = 0, postPhase = 0;
float temp = 0, press = 0;
bool running = false, _menu = false, _setting1 = false, _setting2 = false, door_open = true;
int time = 0, Ttime = 0;
unsigned int _stop = 0;

typedef struct {
    float max_press,  preHeat, strile_temp,
          preVacMin_val[3], preVacMax_val[3], postVac_val[3];
unsigned int ELprv, ELpsv, time_out, preVac_time[3], postVac_time[3], strile_time, _sp;
} Var,Save_var;
 
eeprom Save_var save_normal, save_fast, save_b_d, save_m121;
Var normal, fast, b_d, m121;
float temperature();
float pressure();
void Timing();
void Error(int i)
{
    running = false;
    el = 0;
    vv = 0;
    vp = 0;
    wp = 0;
    wv = 0; 
    phase = 0;
    prePhase = 0;
    postPhase = 0;
    Err_LED = 1;
    time = 4;
    buzz_on;
    while (1)
    {         
        if((press>0.1 || press<-0.1) && door_open){
            dl_close1;
            dl_close2;
            delay_ms(2000);
            dl_off1;
            dl_off2;      
            door_open=false;
        }else if(press<0.05 && press>-0.05 && !door_open){
            dl_open1;
            dl_open2;
            delay_ms(2000);
            dl_off1;
            dl_off2;   
            door_open=true;
        }
        if (press > 0.05)
        {
            out= 1;
            bal_on;
        }
        else if(press < 0.04)
        {
            out= 0;
            bal_off;
        }
        temp = temperature();
        press = pressure();
        lcd_clear();
        sprintf(string, "%.1f %cC", temp, 223);
        lcd_gotoxy(0, 0);
        lcd_puts(string);
        sprintf(string, "|%5.2f bar", press);
        lcd_gotoxy(10, 0);
        lcd_puts(string);

        sprintf(string, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
        , 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255);
        lcd_gotoxy(0, 1);
        lcd_puts(string);

        sprintf(string, "--Error  %03d--", i);
        lcd_gotoxy(3, 2);
        lcd_puts(string);

        sprintf(string, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
        , 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255);
        lcd_gotoxy(0, 3);
        lcd_puts(string);
        delay_ms(100);
        Timing();
        if (set || back) goto exit;
        if (time <1) buzz_off;
    }
    exit:
    buzz_off;
    Err_LED = 0;
    delay_ms(200);
}

unsigned int read_adc(unsigned char adc_input)
{
    ADMUX = adc_input | ADC_VREF_TYPE;
    // Delay needed for the stabilization of the ADC input voltage
    delay_us(10);
    // Start the AD conversion
    ADCSRA |= (1 << ADSC);
    // Wait for the AD conversion to complete
    while ((ADCSRA & (1 << ADIF)) == 0) ;
    ADCSRA |= (1 << ADIF);
    return ADCW;
}

int Read_MAX31865()
{
    int low = 0, high = 0, data;

    SET_SPID_SS_LOW;
    delay_ms(5);

    SPDR = 0x01;
    while (!(SPSR & 0x80)) ;
    SPDR = 0xFF;
    while (!(SPSR & 0x80)) ;
    high = SPDR;
    SET_SPID_SS_HIGH;

    delay_ms(5);

    SET_SPID_SS_LOW;
    delay_ms(5);

    SPDR = 0x02;
    while (!(SPSR & 0x80)) ;
    SPDR = 0xFF;
    while (!(SPSR & 0x80)) ;
    low = SPDR;
    SET_SPID_SS_HIGH;

    data = high;
    data = (data << 8);
    data = (data | low);

    return data;
}

void max_init()
{
    PORTB = 0XFF;      //////////////// in khat bayad dorost beshe
    SET_SPID_SS_LOW;
    delay_ms(5);

    SPDR = 0x80;
    while (!(SPSR & 0x80)) ;
    SPDR = 0xC1;
    while (!(SPSR & 0x80)) ;
    SET_SPID_SS_HIGH;
}

float calc_temp_pt100(int res)
{
    float re = (float)res;
    float a = 0.00390830;
    float b = 0.0000005775;
    float R = (re * 430.0) / 65536.000000;
    float T = ((-a) + sqrt((a * a) + (4.0 * b * ((R - 100.0) / 100.000000)))) / (2.0 * b);
    return T;
}

float pressure()
{
    float press = ((((float)read_adc(7)) * adc_ref_Volt) / 512.0) - 2.0;
    return press + press_off;
}

float temperature()
{
    static int t;
    static float temp_avg[temp_avg_count + 1];
    float sum = 0;
    int i;
    temp_avg[t] = calc_temp_pt100(Read_MAX31865());  
    
    for (i = 0; i < temp_avg_count; i++)
        sum += temp_avg[i];
    if (++t >= temp_avg_count) t = 0;
    sum /= temp_avg_count;
    sum += ((sum-30.0)/12.5);        
    return  sum + temp_off;
}



void Test()
{
    static unsigned int pos = 0, state = 0, delay = 0;
    lcd_clear();   
    sprintf(string, "%.1f %cC", temp, 223);
    lcd_gotoxy(0, 0);
    lcd_puts(string);
    sprintf(string, "|%5.2f bar", press);
    lcd_gotoxy(9, 0);
    lcd_puts(string);                                 
    switch (pos)
    {
        case 0:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Water Pump");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: WP");
            lcd_gotoxy(0, 3);
            if (state % 2)
            {
                wp = 1;
                sprintf(string, "status: ON");
            }
            else
            {
                wp = 0;
                sprintf(string, "status: OFF");
            }
            lcd_puts(string);
            break;
        case 1:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Water Valve");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: WV");
            lcd_gotoxy(0, 3);
            if (state % 2)
            {
                wv = 1;
                sprintf(string, "status: ON");
            }
            else
            {
                wv = 0;
                sprintf(string, "status: OFF");
            }
            lcd_puts(string);
            break;
        case 2:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Vacuum Pump");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: VP");
            lcd_gotoxy(0, 3);
            if (state % 2)
            {
                vp = 1;
                sprintf(string, "status: ON");
            }
            else
            {
                vp = 0;
                sprintf(string, "status: OFF");
            }
            lcd_puts(string);
            break;
        case 3:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Vacuum Valve");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: VV");
            lcd_gotoxy(0, 3);
            if (state % 2)
            {
                vv = 1;
                sprintf(string, "status: ON");
            }
            else
            {
                vv = 0;
                sprintf(string, "status: OFF");
            }
            lcd_puts(string);
            break;
        case 4:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Output Valve");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: OUT");
            lcd_gotoxy(0, 3);
            if (state % 2)
            {
                out= 1;
                sprintf(string, "status: ON");
            }
            else
            {
                out= 0;
                sprintf(string, "status: OFF");
            }
            lcd_puts(string);
            break;
        case 5:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Heater Element");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: EL");
            lcd_gotoxy(0, 3);
            if (state % 2)
            {
                el = 1;
                sprintf(string, "status: ON");
            }
            else
            {
                el = 0;
                sprintf(string, "status: OFF");
            }
            lcd_puts(string);
            break;
        case 6:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Balance Valve");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: BAL");
            lcd_gotoxy(0, 3);
            if (state % 2)
            {
                bal_on;
                sprintf(string, "status: ON");
            }
            else
            {
                bal_off;
                sprintf(string, "status: OFF");
            }
            lcd_puts(string);
            break;
        case 7:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Door Lock");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: DL");
            lcd_gotoxy(0, 3);
            Timing();
            if (state % 4 == 0)
            {   
                if(time<1){
                    dl_off1;
                    dl_off2;
                } 
            }
            else if (state % 4 == 1)
            { 
                dl_off1;
                dl_off2;
                dl_close1;
                dl_close2; 
                time=4;
                state++;
                sprintf(string, "status: CLOSE");
            }
            else if (state % 4 == 2)
            {
                if(time<1){
                    dl_off1;
                    dl_off2;
                } 
            }
            else
            {   
                dl_off1;
                dl_off2;
                dl_open1;
                dl_open2; 
                time=4;
                state++;
                sprintf(string, "status: OPEN");
            }
            lcd_puts(string);
            break;
        case 8:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Buzzer");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: Buzz");
            lcd_gotoxy(0, 3);
            if (state % 2)
            {
                buzz_on;
                sprintf(string, "status: ON");
            }
            else
            {
                buzz_off;
                sprintf(string, "status: OFF");
            }
            lcd_puts(string);
            break;
        case 9:
            lcd_gotoxy(0, 1);
            lcd_puts("Name: Error LED");
            lcd_gotoxy(0, 2);
            lcd_puts("Symbol: Err");
            lcd_gotoxy(0, 3);
            if (state % 2)
            {
                Err_LED = 1;
                sprintf(string, "status: ON");
            }
            else
            {
                Err_LED = 0;
                sprintf(string, "status: OFF");
            }
            lcd_puts(string);
            break;

        default:
            break;
    }
    if (delay < 2)
    {
        if (up)
        {
            pos = (pos < 9) ? pos + 1 : 0;
            state = 0;
            delay = 3;
        }
        if (down)
        {
            pos = (pos > 0) ? pos - 1 : 9;
            state = 0;
            delay = 3;
        }
        if (left || right)
        {
            if (++state > 100) state = 0;
            delay = 3;
        }
    }
    else delay--;

    if (back || set)
    {
        el = 0;
        vv = 0;
        vp = 0;
        wp = 0;
        wv = 0;   
        out= 0;
        dl_off1;
        dl_off2;
        Err_LED = 0;
        bal_off;
        buzz_off;
        _setting2 = false;
    }
}

void Reset_all()
{   
    (&normal)->max_press = 2.20; 
    (&normal)->ELprv = 15;
    (&normal)->ELpsv = 15;
    (&normal)->preHeat = 100;
    (&normal)->strile_time = 270;
    (&normal)->strile_temp = 134.5;
    (&normal)->time_out = 3600;
    (&normal)->_sp = 20;

    (&normal)->preVacMin_val[0] = -0.7;
    (&normal)->preVac_time[0] = 120;
    (&normal)->preVacMax_val[0] = 0.2;
    
    (&normal)->postVac_val[0] = -0.5;
    (&normal)->postVac_time[0] = 120;

    (&normal)->preVacMin_val[1] = -0.6;
    (&normal)->preVac_time[1] = 120;
    (&normal)->preVacMax_val[1] = 0.3;
    
    (&normal)->postVac_val[1] = -0.6;
    (&normal)->postVac_time[1] = 120;

    (&normal)->preVacMin_val[2] = -0.5;
    (&normal)->preVac_time[2] = 120;    
    (&normal)->preVacMax_val[2] = 0.4;
    
    (&normal)->postVac_val[2] = -0.7;
    (&normal)->postVac_time[2] = 120; 
    
    
    (&fast)->max_press = 1.25;
    (&fast)->ELprv = 15;
    (&fast)->ELpsv = 15;
    (&fast)->preHeat = 100;
    (&fast)->strile_time = 60;
    (&fast)->strile_temp = 121.5;
    (&fast)->time_out = 2400;
    (&fast)->_sp = 20;

    (&fast)->preVacMin_val[0] = -0.7;
    (&fast)->preVac_time[0] = 120;
    (&fast)->preVacMax_val[0] = 0.2;
    
    (&fast)->postVac_val[0] = -0.5;
    (&fast)->postVac_time[0] = 120;

    (&fast)->preVacMin_val[1] = -0.6;
    (&fast)->preVac_time[1] = 120;
    (&fast)->preVacMax_val[1] = 0.3;
    
    (&fast)->postVac_val[1] = -0.6;
    (&fast)->postVac_time[1] = 120;

    (&fast)->preVacMin_val[2] = -0.5;
    (&fast)->preVac_time[2] = 120;    
    (&fast)->preVacMax_val[2] = 0.4;
    
    (&fast)->postVac_val[2] = -0.7;
    (&fast)->postVac_time[2] = 120;

    
    (&m121)->max_press = 1.2;
    (&m121)->ELprv = 15;
    (&m121)->ELpsv = 15;
    (&m121)->preHeat = 100;
    (&m121)->strile_time = 900;
    (&m121)->strile_temp = 121.5;
    (&m121)->time_out = 5400;
    (&m121)->_sp = 20;

    
    (&m121)->preVacMin_val[0] = -0.7;
    (&m121)->preVac_time[0] = 120;
    (&m121)->preVacMax_val[0] = 0.2;
    
    (&m121)->postVac_val[0] = -0.5;
    (&m121)->postVac_time[0] = 120;

    (&m121)->preVacMin_val[1] = -0.6;
    (&m121)->preVac_time[1] = 120;
    (&m121)->preVacMax_val[1] = 0.3;
    
    (&m121)->postVac_val[1] = -0.6;
    (&m121)->postVac_time[1] = 120;

    (&m121)->preVacMin_val[2] = -0.5;
    (&m121)->preVac_time[2] = 120;    
    (&m121)->preVacMax_val[2] = 0.4;
    
    (&m121)->postVac_val[2] = -0.7;
    (&m121)->postVac_time[2] = 120;

    (&b_d)->max_press = 1.25;
    (&b_d)->ELprv = 15;
    (&b_d)->ELpsv = 15;
    (&b_d)->preHeat = 100;
    (&b_d)->strile_time = 60;
    (&b_d)->strile_temp = 121.5;
    (&b_d)->time_out = 2400;
    (&b_d)->_sp = 30;

    
    (&b_d)->preVacMin_val[0] = -0.7;
    (&b_d)->preVac_time[0] = 60;
    (&b_d)->preVacMax_val[0] = 0.3;
    (&b_d)->postVac_val[0] = -0.7;
    (&b_d)->postVac_time[0] = 60;

    (&b_d)->preVacMin_val[1] = 0;
    (&b_d)->preVac_time[1] = 0;
    (&b_d)->preVacMax_val[1] = 0;
    (&b_d)->postVac_val[1] = 0;
    (&b_d)->postVac_time[1] = 0;

    (&b_d)->preVacMin_val[2] = 0;
    (&b_d)->preVac_time[2] = 0;
    
    (&b_d)->preVacMax_val[2] = 0;
    (&b_d)->postVac_val[2] = 0;
    (&b_d)->postVac_time[2] = 0; 
            
    if(complete<=0)complete=0;
    _mode=0;
    temp_off = 0;
    press_off = 0;
}

void Reset()
{
    static bool pos = false;
    lcd_clear();
    if (left || right) pos = !pos;
    
     
    if (pos){
        lcd_gotoxy(0, 1);
        lcd_puts("->All setting reset");
        lcd_gotoxy(0, 2);
        lcd_puts("  Cycle reset");
    }else{                  
        lcd_gotoxy(0, 1);
        lcd_puts("  All setting reset");
        lcd_gotoxy(0, 2);
        lcd_puts("->Cycle reset");
    }    
    lcd_gotoxy(0, 4);     
    sprintf(string,"----Version %4.2f----",ver);
    lcd_puts(string);
    if (set)
        if (pos)
        {
            Reset_all();
            lcd_clear();
            lcd_gotoxy(0, 1);
            lcd_puts(" All settings are \n reset to default \n     values  ");
            delay_ms(2000);
            _setting2 = false;
        }
        else{
            complete=0;
            lcd_clear();
            lcd_gotoxy(0, 1);
            lcd_puts(" Cycle counter \n reset to zero");
            delay_ms(2000);
            _setting2 = false;
        }
    if (back) _setting2 = false; 
}

void save()
{
    save_fast = fast;
    save_b_d = b_d;
    save_m121 = m121;
    save_normal = normal;
}

void load()
{
    normal = save_normal;
    fast = save_fast;
    b_d = save_b_d;
    m121 = save_m121;
}

void lcd()
{
    lcd_clear();
    sprintf(string, "%.1f %cC", temp, 223);
    lcd_gotoxy(0, 0);
    lcd_puts(string);
    sprintf(string, "|%5.2f bar", press);
    lcd_gotoxy(9, 0);
    lcd_puts(string);

    sprintf(string, "%02d:%02d:%02d", hour, minute, second);
    lcd_gotoxy(0, 1);
    lcd_puts(string);
    if(running)  
    sprintf(string, "| %02d:%02d", Ttime/60,Ttime%60);  
    else sprintf(string, "|"); 
    lcd_gotoxy(9, 1);
    lcd_puts(string);

    sprintf(string, "%04d", complete);
    lcd_gotoxy(0, 2);
    lcd_puts(string);         
    sprintf(string, "| %s", status[phase]);
    lcd_gotoxy(9, 2);
    lcd_puts(string);

    sprintf(string, "%s", mode[_mode]);
    lcd_gotoxy(0, 3);
    lcd_puts(string);
    //if(time>0 && running && prePhase==2 && (phase==4 || phase==5 || phase==6))sprintf(string, "| %d", -(time % var->_sp) );      
    if(time>0 && running) sprintf(string, "| %d",time);
    else       sprintf(string, "|",time); 
    
    lcd_gotoxy(9, 3);
    lcd_puts(string);
}

void Clock_set()
{
    static bool z = false;
    int p = 0;
    lcd_clear();
    lcd_puts("   Clock  Setting");
    if (back)
    {
        _setting2 = false; 
        z = false;
        p = 0; 
    }
    if (set)
    {
        _setting2 = false;
        rtc_set_time(hh, mm, second);
        rtc_set_time(yy, _mm, dd);
    }
    if (right && p<4) p++;
    if (left && p>0) p--;
    switch(p)
    {   
        case 0:
            if (up)
                if (hh >= 23) hh = 0;
                else hh++;

            if (down)
                if (hh <= 0) hh = 23;
                else hh--;

            if (z) sprintf(string, "%02d:%02d:%02d", hh, mm, second);
            else sprintf(string, "  :%02d:%02d", mm, second);
            lcd_gotoxy(6, 2);
            lcd_puts(string);
            sprintf(string,"%04d-%02d-%02d", yy, _mm, dd);
            lcd_gotoxy(5, 3);  
            lcd_puts(string);           
        break;
        case 1:
            if (up)
                if (mm >= 59) mm = 0;
                else mm++;

            if (down)
                if (mm <= 0) mm = 59;
                else mm--;

            if (z) sprintf(string, "%02d:%02d:%02d", hh, mm, second);
            else sprintf(string, "%02d:  :%02d", hh, second);
            lcd_gotoxy(6, 2);
            lcd_puts(string);   
            sprintf(string,"%04d-%02d-%02d", yy, _mm, dd);
            lcd_gotoxy(5, 3);  
            lcd_puts(string);   
        break; 
        case 2:
            if (up) yy++;

            if (down)
                if (yy <= 0) yy = 2100;
                else yy--;

            if (z) sprintf(string, "%04d-%02d-%02d", yy, _mm, dd);
            else sprintf(string, "    -%02d-%02d", _mm, dd);   
            lcd_gotoxy(5, 3); 
            lcd_puts(string); 
            sprintf(string,"%02d:%02d:%02d", hh, mm, second);
            lcd_gotoxy(6, 2);  
            lcd_puts(string);   
            break; 
        case 3:
            if (up)
                if (_mm > 12) _mm = 0;
                else _mm++;

            if (down)
                if (_mm <= 0) _mm = 12;
                else _mm--;

            if (z) sprintf(string, "%04d-%02d-%02d", yy, _mm, dd);
            else sprintf(string, "%04d-  -%02d", yy, dd);   
            lcd_gotoxy(5, 3); 
            lcd_puts(string); 
            sprintf(string,"%02d:%02d:%02d", hh, mm, second);
            lcd_gotoxy(6, 2);  
            lcd_puts(string);   
            break;
        case 4:
            if (up)
                if (dd >= 31) dd = 0;
                else dd++;

            if (down)
                if (dd <= 0) dd = 31;
                else dd--;

            if (z) sprintf(string, "%04d-%02d-%02d", yy, _mm, dd);
            else sprintf(string, "%04d-%02d-  ", yy, _mm );   
            lcd_gotoxy(5, 3); 
            lcd_puts(string); 
            sprintf(string,"%02d:%02d:%02d", hh, mm, second);
            lcd_gotoxy(6, 2);  
            lcd_puts(string);   
            break;        
        default: break;
    }
    z = !z;
}

void Setting(Var* var)
{
    static int y = 0, page = 0, preVac = 0, postVac = 0;
    static float c1 = 0.01;
    static int c2 = 1;
    lcd_clear();
    if (left || right)
    {
        c1 += 0.01;
        c2 += 1;
    }
    else
    {
        c1 = 0.0;
        c2 = 0;
    }
    switch (page)
    {
        case 0:
            switch (y)
            {
                case 0:
                    if (right) var->max_press += c1;
                    if (left) var->max_press -= c1;
                    if (down) y = 1;
                    if (up) y = 3;
                    break;
                case 1:  
                    if (right) var->preHeat += c1;
                    if (left) var->preHeat -= c1;
                    if (down) y = 3;
                    if (up) y = 1; 
                    break;
                case 2: 
                    if (right) var->ELprv += c2;
                    if (left) var->ELprv -= c2;
                    if (down) y = 2;
                    if (up) y = 0;
                    break;
                case 3: 
                    if (right) var->ELpsv += c2;
                    if (left) var->ELpsv -= c2;
                    if (down) y = 2;
                    if (up) y = 0;
                    break;
                default:
                    break;
            }

            sprintf(string, "max press.: %.2f", var->max_press);
            lcd_gotoxy(2, 0);
            lcd_puts(string);

            sprintf(string, "pre Heat:  %.2f", var->preHeat);
            lcd_gotoxy(2, 1);
            lcd_puts(string); 
            
            sprintf(string, "EL prv:     %d", var->ELprv);
            lcd_gotoxy(2, 2);
            lcd_puts(string);  
            
            sprintf(string, "EL psv:     %d", var->ELpsv);
            lcd_gotoxy(2, 3);
            lcd_puts(string);   
            
            
            break;
        case 1:
            switch (y)
            {
                case 0:
                    if (right) var->strile_time += c2;
                    if (left && var->strile_time) var->strile_time -= c2;
                    if (down) y = 1;
                    if (up) y = 3;
                    break;
                case 1:
                    if (right) var->strile_temp += c1;
                    if (left) var->strile_temp -= c1;
                    if (down) y = 2;
                    if (up) y = 0;
                    break;
                case 2:
                    if (right) var->_sp += c2;
                    if (left && var->_sp > 3) var->_sp -= c2;
                    if (down) y = 3;
                    if (up) y = 1;
                    break;
                case 3:
                    if (right) var->time_out += c2;
                    if (left && var->time_out > 0) var->time_out -= c2;
                    if (down) y = 0;
                    if (up) y = 2;
                    break;
                default:
                    break;
            }

            sprintf(string, "StrileTime: %d", var->strile_time);
            lcd_gotoxy(2, 0);
            lcd_puts(string);

            sprintf(string, "StrileTemp:%.2f", var->strile_temp);
            lcd_gotoxy(2, 1);
            lcd_puts(string);

            sprintf(string, "water (1/n): %d", var->_sp);
            lcd_gotoxy(2, 2);
            lcd_puts(string); 
            
            sprintf(string, "time out(s): %d", var->time_out);
            lcd_gotoxy(2, 3);
            lcd_puts(string);

            break;
        case 2:
        case 3:
        case 4:
            preVac = page - 2;
            switch (y)
            {
                case 0:
                    break;
                case 1:
                    if (right) var->preVacMin_val[(int)preVac] += c1;
                    if (left) var->preVacMin_val[(int)preVac] -= c1;
                    if (down) y = 2;
                    if (up) y = 3;
                    break;
                case 2:
                    if (right) var->preVac_time[(int)preVac] += c2;
                    if (left && var->preVac_time[(int)preVac] > 0) var->preVac_time[(int)preVac] -= c2;
                    if (down) y = 3;
                    if (up) y = 1;
                    break;
                case 3:
                    if (right) var->preVacMax_val[(int)preVac] += c1;
                    if (left) var->preVacMax_val[(int)preVac] -= c1;
                    if (down) y = 1;
                    if (up) y = 2;
                    break;
                default:
                    break;
            }

            sprintf(string, "  Pre Vacuum %d", (int)preVac + 1);
            lcd_gotoxy(2, 0);
            lcd_puts(string);

            sprintf(string, "MinValue: %.2f", var->preVacMin_val[(int)preVac]);
            lcd_gotoxy(2, 1);
            lcd_puts(string);

            sprintf(string, "MinTime(s):  %d", var->preVac_time[(int)preVac]);
            lcd_gotoxy(2, 2);
            lcd_puts(string);

            sprintf(string, "MaxValue: %.2f", var->preVacMax_val[(int)preVac]);
            lcd_gotoxy(2, 3);
            lcd_puts(string);


            break;
        case 5:
        case 6:
        case 7:
            postVac = page - 5;
            switch (y)
            {
                case 0:
                    break;
                case 1:
                    if (right) var->postVac_val[(int)postVac] += c1;
                    if (left) var->postVac_val[(int)postVac] -= c1;
                    if (down) y = 2;
                    if (up) y = 2;
                    break;
                case 2:
                    if (right) var->postVac_time[(int)postVac] += c2;
                    if (left && var->postVac_time[(int)postVac] > 0) var->postVac_time[(int)postVac] -= c2;
                    if (down) y = 1;
                    if (up) y = 1;
                    break;
                case 3:
                    break;
                default:
                    break;
            }

            sprintf(string, "  Post Vacuum %d", (int)postVac + 1);
            lcd_gotoxy(2, 0);
            lcd_puts(string);

            sprintf(string, "MinValue: %.2f", var->postVac_val[(int)postVac]);
            lcd_gotoxy(2, 1);
            lcd_puts(string);

            sprintf(string, "MinTime:  %d", var->postVac_time[(int)postVac]);
            lcd_gotoxy(2, 2);
            lcd_puts(string);
            break;
        default:
            break;
    }

    lcd_gotoxy(0, y);
    lcd_puts("->");
    if (up || down || set || back) delay_ms(50);
    if (back)
        if (page > 0) { page--; y = 1; }
        else
        {
            lcd_clear();
            lcd_puts("\n       Saving\n   Please Wait...");
            save();
            delay_ms(200);
            lcd_clear();
            _setting1 = false;
            y = 0; page = 0; preVac = 0; postVac = 0;
            c1 = 0.01;
            c2 = 1;
        }
    if (set)
        if (page < 7) { page++; y = 1; }
        else
        {
            lcd_clear();
            lcd_puts("\n       Saving\n   Please Wait...");
            save();
            delay_ms(200);
            lcd_clear();
            _setting1 = false;
            y = 0; page = 0; preVac = 0; postVac = 0;
            c1 = 0.01;
            c2 = 1;
        }
}

void Offsets()
{
    static bool pos = true;
    static float c = 0.01;
    lcd_clear();
    if (right || left) c += 0.01;
    else c = 0.0;
    if (pos)
    {
        if (right) temp_off += c;
        if (left) temp_off -= c;
        lcd_gotoxy(0, 1);
        lcd_puts("->");
    }
    else
    {
        if (right) press_off += c;
        if (left) press_off -= c;
        lcd_gotoxy(0, 2);
        lcd_puts("->");
    }
    if (set || back)
    {
        _setting2 = false;
        pos = true;
        c = 0.01;
    }
    lcd_gotoxy(6, 0);
    lcd_puts("OFFSETS");
    lcd_gotoxy(2, 1);
    sprintf(string, "Temp. :  %5.2f", temp_off);
    lcd_puts(string);
    lcd_gotoxy(2, 2);
    sprintf(string, "press. : %5.2f", press_off);
    lcd_puts(string);
    if (up || down)
    {
        pos = !pos;
        delay_ms(50);
    }
    //#asm("wdr")   
}

void Menu()
{
    static int x = 0, y = 0, i;

    if (_setting1)
    {
        switch (y)
        {
            case 0:
                Setting(&normal);
                break;
            case 1:
                Setting(&fast);
                break;
            case 2:
                Setting(&b_d);
                break;
            case 3:
                Setting(&m121);
                break;
            default:
                break;
        }
    }
    else if (_setting2)
    {
        switch (y)
        {
            case 0:
                Clock_set();
                break;
            case 1:
                if(running) _setting2=false;
                else        Test();
                break;
            case 2:
                Reset();
                break;
            case 3:
                Offsets();
                break;
            default:
                break;
        }
    }
    else
    {
        if (right || left) x = (x == 0) ? 10 : 0;
        if (down) y = (y >= 3) ? 0 : y + 1;
        if (up) y = (y <= 0) ? 3 : y - 1;
        if (back)
        {
            _menu = false;
            _setting1 = false;
            _setting2 = false;
            x = 0; y = 0; i = 0;
            lcd_clear();
        }
        if (set && !_setting1 && !_setting2)
        {
            delay_ms(100);
            while (set) ;
            if (x == 0)
            {
                _setting1 = true;
                _setting2 = false;
            }
            else
            {
                _setting1 = false;
                _setting2 = true;
                hh = hour;
                mm = minute; 
                yy = year;
                _mm = month;
                dd = day;
            }

        }
        lcd_clear();
        for (i = 0; i < 4; i++)
        {
            lcd_gotoxy(2, i);
            sprintf(string, "%s", mode[i]);
            lcd_puts(string);

            lcd_gotoxy(2, i);
            sprintf(string, "%s", mode[i]);
            lcd_puts(string);

            lcd_gotoxy(2, i);
            sprintf(string, "%s", mode[i]);
            lcd_puts(string);

            lcd_gotoxy(2, i);
            sprintf(string, "%s", mode[i]);
            lcd_puts(string);
        }


        lcd_gotoxy(9, 0);
        lcd_puts("|  Clock");

        lcd_gotoxy(9, 1);
        lcd_puts("|  Test");

        lcd_gotoxy(9, 2);
        lcd_puts("|  Reset");

        lcd_gotoxy(9, 3);
        lcd_puts("|  Offset");

        lcd_gotoxy(x, y);
        lcd_puts("->");
        // #asm("wdr") 
    }
}

void finish()
{
    buzz_on;
    time=4;
    while (1)
    {    
        if((press>0.1 || press<-0.1) && door_open){
            dl_close1;
            dl_close2;
            delay_ms(2000);
            dl_off1;
            dl_off2;      
            door_open=false;
        }else if(press<0.05 && press>-0.05 && !door_open){
            dl_open1;
            dl_open2;
            delay_ms(2000);
            dl_off1;
            dl_off2;   
            door_open=true;
        }
        if (press > 0.05)
        {
            out= 1;
            bal_on;
        }
        else if(press < 0.04)
        {
            out= 0;
            bal_off;
        }
        lcd_clear();
        sprintf(string, "%.1f %cC", temp, 223);
        lcd_gotoxy(0, 0);
        lcd_puts(string);
        sprintf(string, "|%5.2f bar", press);
        lcd_gotoxy(10, 0);
        lcd_puts(string);

        sprintf(string, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
        , 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255);
        lcd_gotoxy(0, 1);
        lcd_puts(string);

        sprintf(string, "Cycle Complete", press);
        lcd_gotoxy(3, 2);
        lcd_puts(string);

        sprintf(string, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
        , 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255);
        lcd_gotoxy(0, 3);
        lcd_puts(string);
        delay_ms(100);
        Timing();
        if (set || back || !mdo) goto exit;
        if (time < 1) buzz_off;
    }
    exit:  
    buzz_off;
    delay_ms(200);
}

void prevac(int i, Var* var)
{ 
    static int st;
    if (temp < (var->strile_temp - var->ELprv)) el = 1;
    else if (temp > (var->strile_temp - var->ELprv + 0.1)) el = 0;
    switch (prePhase)
    {
        case 0:
            el = 1;
            if (temp > var->preHeat)
            {    
                time = var->preVac_time[i];
                prePhase++;
            }
            break;
        case 1:     
            if (time < var->preVac_time[i]-120 && press>-0.1) Error(3); 
            if (press > (var->preVacMin_val[i]) && time > 1)
            {
                vv = 1;
                vp = 1;
            }
            else
            {
                vv = 0;
                vp = 0;
                time = var->_sp;
                st=Ttime;
                prePhase++;
            }
            break;
        case 2:
            if (time < 1) time = var->_sp;
            if (Ttime-st>900) Error(4); 
            if (press < (var->preVacMax_val[i]))
            {
                if (time > var->_sp - 1)   
                {
                    wv = 1;
                    wp = 1;
                }
                else
                {
                    wv = 0;
                    wp = 0;
                }
            }
            else
            {
                wv = 0;
                wp = 0;
                prePhase++;
            }
            break;
        case 3:
            if (press > 0.2 && var->preVac_time[i + 1] != 0 && i<2) out= 1; 
            else{ 
                out= 0;
                prePhase++;
            }
            break;
        default: break;
    }
}

void postvac(int i, Var* var)
{  
    if (temp < (var->strile_temp - var->ELpsv)) el = 1;
    else if (temp > (var->strile_temp - var->ELpsv + 0.1)) el = 0;

    switch (postPhase)
    {
        case 0:      
            bal_on;
            if (press > 0.2) out= 1;
            else{
                out= 0;
                postPhase++;
                time = var->postVac_time[i];
            }
            break;
        case 1:
            if (time < var->postVac_time[i]-120 && press>-0.1) Error(3); 
            if (press > (var->postVac_val[i]) && time > 0)
            {
                vv = 1;
                vp = 1;
            }
            else
            {
                vv = 0;
                vp = 0;
                postPhase++; 
                time=0;
            }
            break;
        case 2:           
            bal_off;
            if (time < -120) Error(7);
            if (press > -0.05)                
                postPhase++;
            break;
        default: break;
    }
}

void Start(Var* var)
{
    static bool flag = false;
    static int t;  
    if((temp>var->strile_temp+5.0) || (press>var->max_press+0.2)) Error(9);  
    if(Ttime>var->time_out) Error(10);
    switch (phase)
    {
        case 0:
            if (!mdo) Error(1);
            else
            {
                dl_close1;
                dl_close2;
                bal_on;
                if (time < 1)
                {
                    phase++;
                    dl_off1;
                    dl_off2;
                    door_open=false;
                }
            }
            break;
        case 1:
            if (press > 0.05)  out= 1;  
            else{
                out= 0;
                time = 0;
                phase++;
            }
            break;
        case 2:
            bal_on;
            if (press > -0.05)
            {
                bal_off;
                time = 0; 
                phase++;
            }
            break;
        case 3:  
            if (time < -1800) Error(2);
            if (temp < var->preHeat) el = 1;
            else{ 
                time=0;  
                bal_on;
                phase++;
            }  
            break;
        case 4:
            if (var->preVac_time[0] != 0) prevac(0, var);
            else phase++;
            if (prePhase > 3)
            {
                prePhase = 0;
                phase++;
                time = 0;
            }
            break;
        case 5:
            if (var->preVac_time[1] != 0) prevac(1, var);
            else phase++;
            if (prePhase > 3)
            {
                prePhase = 0;
                phase++;
                time = 0;
            }
            break;
        case 6:
            if (var->preVac_time[2] != 0) prevac(2, var);
            else phase++;
            if (prePhase > 3)
            {
                prePhase = 0;
                phase++;
                time = 0;
            }
            break;
        case 7:  
            el=1;
            phase++;
            break;
        case 8:     
            if(time < 1) time = var->_sp;
            if (press > var->max_press+0.1)//{
                out = 1;
               // flag = 1; }
            else if (press < var->max_press)//{
                out= 0;
              /*  if (flag == 1) flag = 2;
            }
            if (flag == 2)
            {
                wp = 1;
                wv = 1;
                delay_ms(500);
                wp = 0;
                wv = 0;
                flag = 0;
            }     */
            if (time > var->_sp - 1)
            {
                wp = 1;
                wv = 1;
            }
            else
            {
                wp = 0;
                wv = 0;
            }
            
            if (temp >= var->strile_temp)
            {
                wv = 0;
                wp = 0;
                time = var->strile_time;
                phase++;
            }
            break;
        case 9:
            if (press > var->max_press+0.1)//{
                out= 1;
                //flag = 1; }
            else if (press < var->max_press)//{
                out= 0;
               /* if (flag == 1) flag = 2;
            }
            if (flag == 2)
            {
                wp = 1;
                wv = 1;
                delay_ms(1000);
                wp = 0;
                wv = 0;
                flag = 0;
            }else */
            if (press < var->max_press-0.1)
            {
                if (time % var->_sp == 0)
                {
                    wp = 1;
                    wv = 1;
                }
                else
                {
                    wp = 0;
                    wv = 0;
                }
            }
            if ((temp < var->strile_temp - 1.0) || (press < var->max_press - 0.1)){
                flag=true;
                t=Ttime;
            }else flag=false; 
            
            if(flag && Ttime-t>120) Error(6);
                
            if (temp > var->strile_temp + 0.3) el = 0;
            else if (temp < var->strile_temp) el = 1;
            if (time < 1) phase++;
            break;
        case 10:   
            time = 0;
            phase++;
            break;
        case 11:
            el = 0;
            wv = 0;
            wp = 0;   
            if (time < -180) Error(8);
            if (press > 0.2) out= 1;
            else{
                out= 0;
                phase++;
                time = 0;
            }
            break;
        case 12:
            if (var->postVac_time[0] != 0) postvac(0, var);
            else phase++;
            if (postPhase > 2)
            {
                postPhase = 0;
                phase++;
                time = 0;
            }
            break;
        case 13:
            if (var->postVac_time[1] != 0) postvac(1, var);
            else phase++;
            if (postPhase > 2)
            {
                postPhase = 0;
                phase++;
                time = 0;
            }
            break;
        case 14:
            if (var->postVac_time[2] != 0) postvac(2, var);
            else phase++;
            if (postPhase > 2)
            {
                postPhase = 0;
                phase++;
                time = 4;
            }
            break;
        case 15:
            dl_open1;
            dl_open2;
            if (time < 1)
            {  
                phase = 0;
                prePhase = 0;
                postPhase = 0;
                complete++;
                running = false;
                dl_off1;
                dl_off2;
                door_open = true;   
                finish();
            }
            break;
        default: break;
    }

}

void Timing()
{
    static int save_t;
    rtc_get_time(&hour, &minute, &second); 
    rtc_get_date(0,&day, &month, &year);
    if (save_t != second)
    {
        if (--time < -9999) time = 0; 
        if (!running || ++Ttime > 9999 || phase<4) Ttime = 0;  
        
        save_t = second;
        lcd_init(20);
        lcd_clear();
    }
}

void main(void)
{
    /* 
    // Watchdog Timer initialization
    // Watchdog Timer Prescaler: OSC/2048k
    #pragma optsize-
    WDTCR=(1<<WDCE) | (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
    WDTCR=(0<<WDCE) | (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
    #ifdef _OPTIMIZE_SIZE_
    #pragma optsize+
    #endif 
    */

    // Input/Output Ports initialization
    // Port A initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
    DDRA = (0 << DDA7) | (1 << DDA6) | (1 << DDA5) | (1 << DDA4) | (1 << DDA3) | (1 << DDA2) | (1 << DDA1) | (1 << DDA0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
    PORTA = (0 << PORTA7) | (1 << PORTA6) | (1 << PORTA5) | (1 << PORTA4) | (1 << PORTA3) | (1 << PORTA2) | (1 << PORTA1) | (1 << PORTA0);

    // Port B initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=In Bit2=Out Bit1=Out Bit0=Out 
    DDRB = (1 << DDB7) | (1 << DDB6) | (1 << DDB5) | (1 << DDB4) | (0 << DDB3) | (1 << DDB2) | (1 << DDB1) | (1 << DDB0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
    PORTB = (0 << PORTB7) | (0 << PORTB6) | (0 << PORTB5) | (0 << PORTB4) | (0 << PORTB3) | (0 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);

    // Port C initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
    DDRC = 0x55;
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
    PORTC = 0xAA;

    // Port D initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
    DDRD = 0b11011000;
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
    PORTD = 0x00;

    // Port E initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
    DDRE = (1 << DDE7) | (1 << DDE6) | (1 << DDE5) | (1 << DDE4) | (0 << DDE3) | (0 << DDE2) | (1 << DDE1) | (1 << DDE0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
    PORTE = (0 << PORTE7) | (0 << PORTE6) | (0 << PORTE5) | (0 << PORTE4) | (1 << PORTE3) | (1 << PORTE2) | (0 << PORTE1) | (0 << PORTE0);

    // Port F initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
    DDRF = (0 << DDF7) | (0 << DDF6) | (1 << DDF5) | (1 << DDF4) | (1 << DDF3) | (1 << DDF2) | (1 << DDF1) | (1 << DDF0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
    PORTF = (0 << PORTF7) | (0 << PORTF6) | (0 << PORTF5) | (0 << PORTF4) | (0 << PORTF3) | (0 << PORTF2) | (0 << PORTF1) | (0 << PORTF0);

    // Port G initialization
    // Function: Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
    DDRG = (1 << DDG4) | (1 << DDG3) | (1 << DDG2) | (1 << DDG1) | (1 << DDG0);
    // State: Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
    PORTG = (0 << PORTG4) | (0 << PORTG3) | (0 << PORTG2) | (0 << PORTG1) | (0 << PORTG0);
    buzz_on;


    ADMUX = ADC_VREF_TYPE;
    ADCSRA = (1 << ADEN) | (0 << ADSC) | (0 << ADFR) | (0 << ADIF) | (0 << ADIE) | (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    SFIOR = (0 << ACME);

    // SPI initialization
    // SPI Type: Master
    // SPI Clock Rate: 125.000 kHz
    // SPI Clock Phase: Cycle Half
    // SPI Clock Polarity: Low
    // SPI Data Order: MSB First
    SPCR = (0 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) | (0 << CPOL) | (1 << CPHA) | (1 << SPR1) | (0 << SPR0);
    SPSR = (0 << SPI2X);

    // Bit-Banged I2C Bus initialization
    // I2C Port: PORTD
    // I2C SDA bit: 1
    // I2C SCL bit: 0
    // Bit Rate: 100 kHz
    i2c_init();
    rtc_init(0, 0, 0);
    rtc_get_time(&hour, &minute, &second); 
    rtc_get_date(0,&day, &month, &year);
    if (hour > 23 || minute > 59 || second > 59
        || month > 12 || day > 31)  
    {
        rtc_set_time(0, 0, 0);
        rtc_set_date(0, 0, 0, 0);
    }
    // RS - PORTA Bit 1
    // RD - PORTA Bit 0
    // EN - PORTA Bit 3
    // D4 - PORTA Bit 2
    // D5 - PORTA Bit 5
    // D6 - PORTA Bit 4
    // D7 - PORTA Bit 6
    // Characters/line: 20
    lcd_init(20);
    max_init();
    if (!loaded)
    {
        Reset_all();
        save();
        loaded = true;
    }
    else load(); 
    buzz_off;
    while (1)
    {
        temp = temperature();
        press = pressure();
        Timing();
        if (set && !_menu)
        {
            _menu = true;
            lcd_clear();
            delay_ms(100);
            while (set || back) ;
        }
        if (_menu) Menu();
        else
        {
            lcd();
            if (start && !running)
            {
                running = true;
                time = 4;
            }
        } 
       

        
        if (running)
        {
            switch (_mode)
            {
                case 0:
                    Start(&normal);
                    break;
                case 1:
                    Start(&fast);
                    break;
                case 2:
                    Start(&b_d);
                    break;
                case 3:
                    Start(&m121);
                    break;
                default:
                    break;
            }
            if (back) _stop++;
            if (!back) _stop = 0;
            if (_stop > 10){
                _stop = 0;
                running = false;
                el = 0;
                vv = 0;
                vp = 0;
                wp = 0;
                wv = 0;   
                out = 0;
                bal_off;
                phase = 0;
                prePhase = 0;
                postPhase = 0; 
            }
        }
        else if (!_menu)
        {    
            phase = 0;
            prePhase = 0;
            postPhase = 0;   
            if(temp < 60.0) el=1;                
            else if(temp > 60.5) el = 0;
            if((press>0.1 || press<-0.1) && door_open){
                dl_close1;
                dl_close2;
                delay_ms(2000);
                dl_off1;
                dl_off2;      
                door_open=false;
            }else if(press<0.05 && press>-0.05 && !door_open){
                dl_open1;
                dl_open2;
                delay_ms(2000);
                dl_off1;
                dl_off2;   
                door_open=true;
            }
            if (press > 0.05)
            {
                out= 1;
                bal_on;
            }
            else if(press < 0.04)
            {
                out= 0;
                bal_off;
            }
            if (left)
                if (_mode == 0) _mode = 3;
                else _mode--;
            if (right)
                if (_mode == 3) _mode = 0;
                else _mode++;
        }
        delay_ms(120);
        //   #asm("wdr") 
    }

}  