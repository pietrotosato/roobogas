
#include "mbed.h"
#include <string.h>
#include "mcp342x.h"
#include "mcp4716.h"

#define USB_BUFF_SIZE 32

#define CH1         0u
#define CH2         1u
#define VREF	  	1.024f
#define RFB1M		0u
#define RFB1k		1u

int current_DAC[2] = {0,0};
const float RFB[2] = {1000000, 1000};
static uint8_t sw_set[2] = {RFB1k,RFB1M};
MCP342X::PgaSetting pgaSetting[4] = { MCP342X::PGA_1X, MCP342X::PGA_1X, MCP342X::PGA_1X, MCP342X::PGA_1X };

static BufferedSerial serial_port(USBTX, USBRX, 115200);
I2C i2c(I2C_SDA, I2C_SCL);

// gpio configuration
static AnalogIn vh1(A4);
static AnalogIn vh2(A5);
static DigitalOut vrefEn(D9);
static DigitalOut led1(D11);
static DigitalOut led2(D12);
static DigitalOut sw1(D10);
static DigitalOut sw2(D8);

// PwmOut led(LED1);
DigitalOut led0(LED1); // built in led

typedef struct {
    int    setpoint[2]; 
} setpoint_mail_t;

typedef struct {
    float heater_voltage[2];
    float heater_current[2];
    float heater_resistance[2];
    float sensor_voltage[2];
    float sensor_resistance[2];
    int16_t dataraw[4];
} report_mail_t;

Mail<setpoint_mail_t, 16> updatedac_mb;
Mail<report_mail_t, 16> report_mb;
Mail<report_mail_t, 16> pollutionCheck_mb;

Thread readSignals_thd;
Thread updateDAC_thd;
Thread pollutionCheck_thd;

static void processCmd(char *rx);
static int16_t getAdcData(MCP342X *mcp3428, MCP342X::AdcChannel ch, MCP342X::PgaSetting pga);
static void SW_Set(uint8_t _ch, uint8_t _set);

void readSignals_thdFn(void) {
    
    // init HW
    vh1.set_reference_voltage(3.3f);
    vh2.set_reference_voltage(3.3f);
    vrefEn.write(1u);

    // set feedback resistors 
    SW_Set(CH1, RFB1k);
    SW_Set(CH2, RFB1M);
    
    // set amplifier gain
    pgaSetting[CH1] = MCP342X::PGA_8X;
    pgaSetting[CH2] = MCP342X::PGA_8X;

    MCP342X mcp342x(&i2c, MCP342X::SLAVE_ADDRESS_6BH);
    
    const uint8_t gain[4] = {1,2,4,8};
    int16_t data[4];

    while (true) {
        // Measures each channel.
        for (int i=0; i < 4; i++) {
            data[i] = getAdcData(&mcp342x, (MCP342X::AdcChannel)i, pgaSetting[i]);
        }
        
        report_mail_t *mail = report_mb.try_alloc();
        
        mail->dataraw[0] = data[0];
        mail->dataraw[1] = data[1];
        mail->dataraw[2] = data[2];
        mail->dataraw[3] = data[3];

        mail->heater_voltage[CH1] = vh1.read_voltage() / 0.602;
        mail->heater_voltage[CH2] = vh2.read_voltage() / 0.602;
        mail->heater_current[CH1] = 62.5e-6 * data[3] / 200 / 0.3;
        mail->heater_current[CH2] = 62.5e-6 * data[2] / 200 / 0.3;
        mail->heater_resistance[CH1] = mail->heater_voltage[CH1] / mail->heater_current[CH1];
        mail->heater_resistance[CH2] = mail->heater_voltage[CH2] / mail->heater_current[CH2];

        mail->sensor_voltage[CH1] = 62.5e-6 * data[CH1] / gain[pgaSetting[CH1]];
        mail->sensor_voltage[CH2] = 62.5e-6 * data[CH2] / gain[pgaSetting[CH2]];

        if (data[CH1]>0) {
            mail->sensor_resistance[CH1] = RFB[sw_set[CH1]] * VREF / mail->sensor_voltage[CH1];
        } else {
            mail->sensor_resistance[CH1] = RFB[sw_set[CH1]] * VREF / 62.5e-6 * gain[pgaSetting[CH1]];
        }

        if (data[CH2]>0) {
            mail->sensor_resistance[CH2] = RFB[sw_set[CH2]] * VREF / mail->sensor_voltage[CH2];
        } else {
            mail->sensor_resistance[CH2] = RFB[sw_set[CH2]] * VREF / 62.5e-6 * gain[pgaSetting[CH2]];
        }

        report_mb.put(mail);
        ThisThread::sleep_for(1s);
    }
}



void updateDac_thdFn(void) {

    static MCP4716 dac1(MCP4716::MCP4716A3_ADDR, &i2c);
    static MCP4716 dac2(MCP4716::MCP4716A1_ADDR, &i2c);
    static uint16_t setpoint[2];

    while (true) {
        osEvent evt = updatedac_mb.get();
        if (evt.status == osEventMail) {
            setpoint_mail_t *mail = (setpoint_mail_t *)evt.value.p;
            printf("update DAC: %d, %d \n\r", mail->setpoint[CH1], mail->setpoint[CH2]);

            // check values
            if ( (mail->setpoint[CH1] != -1) && (mail->setpoint[CH1] != setpoint[CH1]) ) {
                setpoint[CH1] = mail->setpoint[CH1];
            }
            if ( (mail->setpoint[CH2] != -1) && (mail->setpoint[CH2] != setpoint[CH2]) ) {
                setpoint[CH2] = mail->setpoint[CH2];
            }

            updatedac_mb.free(mail);
        } 
        
        dac1.setValue(setpoint[CH1]);
        current_DAC[CH1] = setpoint[CH1];

        dac2.setValue(setpoint[CH2]);
        current_DAC[CH2] = setpoint[CH2];
        
        ThisThread::sleep_for(50ms);
    }
}

void pollutionCheck_thdFn(void) {

    led1.write(1u);
    led2.write(1u);
    
    while (true) {
        osEvent evt = pollutionCheck_mb.get();
        if (evt.status == osEventMail) {
            report_mail_t *mail = (report_mail_t *)evt.value.p;
            // printf("check pollution...");

            // check values

            // do something
            led1 = !led1;
            led2 = !led2;

            // printf("average pollution is: %d", something);

            pollutionCheck_mb.free(mail);
        } 
        
        
        ThisThread::sleep_for(500ms);
    }
}


int main(void)
{
    printf(
        "Mbed OS version %d.%d.%d\n",
        MBED_MAJOR_VERSION,
        MBED_MINOR_VERSION,
        MBED_PATCH_VERSION
    );
    printf(
        "Roobopoli challenge 2023 - roobogas city air quality control\nbase project v1.0\n");

    i2c.frequency(400);

    // starting threads
    readSignals_thd.start(callback(readSignals_thdFn));
    updateDAC_thd.start(callback(updateDac_thdFn));
    pollutionCheck_thd.start(callback(pollutionCheck_thdFn));

    ThisThread::sleep_for(200ms);

    // read serial USB port
    char bufferIn[USB_BUFF_SIZE];

    // main loop: report measures and reading serial port
    while(true) {
        
        // catch event if received (reporting to serial)
        osEvent evt = report_mb.get();
        if (evt.status == osEventMail) {
            report_mail_t *r = (report_mail_t *)evt.value.p;
            printf("ch1: %.3f V / %.1f Ohm ->%.1f kOhm\t", r->heater_voltage[CH1], r->heater_resistance[CH1], r->sensor_resistance[CH1]/1000);
            printf("ch2: %.3f V / %.1f Ohm ->%.1f kOhm\t", r->heater_voltage[CH2], r->heater_resistance[CH2], r->sensor_resistance[CH2]/1000);
            printf("[%d,%d,%d,%d]\t", r->dataraw[0],r->dataraw[1],r->dataraw[2],r->dataraw[3]);
            printf("[%d,%d]", current_DAC[CH1],current_DAC[CH2]);
            
            // create msg for pollution check
            report_mail_t *mail = pollutionCheck_mb.try_alloc();
            mail->sensor_resistance[CH1] = r->sensor_resistance[CH1]/1000;
            mail->sensor_resistance[CH2] = r->sensor_resistance[CH2]/1000;
            // may want to add other things to the msg?

            pollutionCheck_mb.put(mail);

            printf("\n");
            report_mb.free(r);
        } 

        int len = 0;
        if (serial_port.readable()) {
            led0.write(1);
            len = serial_port.read(bufferIn ,USB_BUFF_SIZE);
            bufferIn[--len] = '\0';
            // printf("got %d : %s\n", len, bufferIn);
            
            processCmd(bufferIn);
            led0.write(0) ;
        }
    }
    // return 0;
}

static void processCmd(char *rx) {
    switch (rx[0]) {
        case 's':
        {
            if (rx[1] == '1') // setpoint 1
			{
				int val;
				sscanf((const char *)&rx[3], (const char *)"%d\n", &val);
				if ((val<1024) && val != (current_DAC[CH1]))
				{
                    // send command to thread
                    setpoint_mail_t *mail = updatedac_mb.try_alloc();
                    mail->setpoint[CH1] = val&0x3FF;
                    mail->setpoint[CH2] = -1;
                    updatedac_mb.put(mail);
				}
			}
			if (rx[1] == '2') // setpoint 2
			{
				int val;
				sscanf((const char *)&rx[3], (const char *)"%d\n", &val);
				if ((val<1024) && val != (current_DAC[CH2]))
				{
                    // send command to thread
					setpoint_mail_t *mail = updatedac_mb.try_alloc();
                    mail->setpoint[CH1] = -1;
                    mail->setpoint[CH2] = val&0x3FF;
                    updatedac_mb.put(mail);
				}
			}
			break;
        }
    }
}

static int16_t getAdcData(MCP342X *mcp3428, MCP342X::AdcChannel ch, MCP342X::PgaSetting pga) {
    // Configure channel and trigger.
    mcp3428->setChannel(ch);
    mcp3428->setConversionMode(MCP342X::ONE_SHOT);
    mcp3428->setSampleSetting(MCP342X::SAMPLE_15HZ_16BIT);
    mcp3428->setPgaSetting(pga);
    mcp3428->trigger();
    // polling data
    MCP342X::Data data;
    do {
        ThisThread::sleep_for(100ms);
        mcp3428->getData(&data);
    } while(data.st == MCP342X::DATA_NOT_UPDATED);
    return data.value;
}

static void SW_Set(uint8_t _ch, uint8_t _set)
{
	if (_ch==CH1) {
        sw1.write(_set);
		sw_set[CH1] = _set;
	} else if (_ch==CH2) {
		sw2.write(_set);
		sw_set[CH2] = _set;
	}
}

void update_DAC(int dac, int* arg_setpoint) {
    if (dac>CH2 || dac<CH1) return;
    const int setpoint = *arg_setpoint;
    if (current_DAC[dac] != setpoint) {
        printf("update ch %d to %d\n", current_DAC[dac], setpoint);
        current_DAC[dac] += 5;
    } else {
        return;
    }
}