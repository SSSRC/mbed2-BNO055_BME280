#include "mbed.h"

#include "BNO055.h"
#include "BME280.h"
#include <cstdio>

Ticker peakchecktime;

void peakcheck();

//PCへのシリアル通信
Serial pc(USBTX, USBRX, 9600);

//mbed LED1
DigitalOut led(LED1);

//両方のセンサーに使用するI2C
I2C ifaceI2C(I2C_SDA, I2C_SCL);

//両方のセンサーをI2Cで使用する
BOARDC_BNO055 sensor1(&ifaceI2C);
BOARDC_BME280 sensor2(&ifaceI2C);

DigitalIn M2S(PA_8);
DigitalOut S2M(PA_11);

double bme280_P = 0.0;
double bme280_T = 0.0;
//double height = ((pow(1013.25/bme280_P , 1/5.257)-1)*(bme280_T+273.15))/0.0065;
double previous_height = 0.0;
//double n = height-previous_height;//相対高度 
int m = 0;//相対高度が連続して負になった回数

int main(){ 

    peakchecktime.attach(peakcheck,0.1);//頂点検知判定を行う,100Hz(CORE参照)

    printf("mbed READY\r\n");//挨拶表示

    wait_ms(1000);
    led = 1;

    printf("Start Comm - -- ---- --------\r\n");

    //I2Cインターフェースで２つのセンサーを使用する(200KHz)
    ifaceI2C.frequency(100000);
    sensor1.initialize(false);
    sensor2.initialize(false);
/*
    //各センサーのチップIDなどを表示するステートメント
    {
        char chipID = sensor1.getChipID();
        char AccChipID = sensor1.getAccChipID();
        char MagChipID = sensor1.getMagChipID();
        char GyroChipID = sensor1.getGyroChipID();
        char bootLoader = sensor1.getBootRevision();

        char bme280chip = sensor2.getChipID();

        printf("BNO055 (9DOF sensor) ---- ---- ---- ---- ---- ---- ---- ---- ----\r\n");
        printf("chipID = 0x%02X, bootLoader = %d\r\n", chipID, bootLoader);
        printf("Acc    = 0x%02X\r\nMag    = 0x%02X\r\nGyro   = 0x%02X\r\n\r\n", AccChipID, MagChipID, GyroChipID);

        printf("BME280 (Temperature, Humidity, Pressure sensor) -- ---- ---- ----\r\n");
        printf("chipID = 0x%02X\r\n\r\n", bme280chip);

    }

    wait_ms(1000);
    led = 0;

    //各センサーの現在の設定などを表示するステートメント
    {
        char mode = sensor1.getOperationMode();
        char sysStatus = sensor1.getSystemStatus();
        char testStatus = sensor1.getSelfTestResultAll();
        char errorStatus = sensor1.getSystemError();
        char calibST[4];
        sensor1.getCalibStatusAll(calibST[0], calibST[1], calibST[2], calibST[3]);

        char bme280mode_hum = sensor2.getCTRL_humidity();
        char bme280mode_meas = sensor2.getCTRL_measuring();
        char bme280mode_conf = sensor2.getConfig();

        printf("BNO055 (9DOF sensor) ---- ---- ---- ---- ---- ---- ---- ---- ----\r\n");
        printf("Mode = 0x%02X, sysStatus = 0x%02X, testStatus = 0x%02X\r\n", mode, sysStatus, testStatus);
        printf("calibSys = %d [%%], calibAcc = %d [%%], calibMag = %d [%%], calibGyro = %d [%%]\r\n", calibST[0], calibST[1], calibST[2], calibST[3]);
        printf("errorStatus = 0x%02X, err = 0x%02X, len = %d\r\n\r\n", errorStatus, sensor1.getIfaceLastError(), sensor1.getIfaceLastLength());

        printf("BME280 (Temperature, Humidity, Pressure sensor) -- ---- ---- ----\r\n");
        printf("CTRL_HUM = 0x%02X, CTRL_MEAS = 0x%02X, CONFIG = 0x%02X\r\n\r\n", bme280mode_hum, bme280mode_meas, bme280mode_conf);
    }*/

    wait_ms(1000);
    led = 1;

    //各センサーの値を格納するための変数宣言
    short dataBox[12];
    float scAcc, scMag, scGyro, scEUL, scTemp;
    float ax, ay, az, mx, my, mz, gx, gy, gz, temp;
    double yaw, roll, pitch;
    float /*bme280_T = 0.0, bme280_P = 0.0,*/ bme280_H = 0.0;
    char bme280_status = 0x00;

    //センサーのRAW値を実際の数値に変換するための倍率を取得する
    scAcc = sensor1.getAccScale();
    scMag = sensor1.getMagScale();
    scGyro = sensor1.getGyroScale();
    scEUL = sensor1.getEulerScale();
    scTemp = sensor1.getTempScale();


    //ボタンが押されるまで繰り返し続ける(Nucleo専用)
    //ボタンがない場合はwhile(1)の無限ループで代用
    while(1){
        //配列dataBoxに、9軸の値とオイラー角(yaw roll pitch)を格納(計12個の値)
        sensor1.get9AxisAndEUL(dataBox);

        //倍率をかけてRaw値を実際の値に変換
        ax = (float)dataBox[0] * scAcc;
        ay = (float)dataBox[1] * scAcc;
        az = (float)dataBox[2] * scAcc;
        mx = (float)dataBox[3] * scMag;
        my = (float)dataBox[4] * scMag;
        mz = (float)dataBox[5] * scMag;
        gx = (float)dataBox[6] * scGyro;
        gy = (float)dataBox[7] * scGyro;
        gz = (float)dataBox[8] * scGyro;
        /*yaw = (float)dataBox[9] * scEUL;
        roll = (float)dataBox[10] * scEUL;
        pitch = (float)dataBox[11] * scEUL;*/

        sensor1.getEulerFromQ(yaw,roll,pitch);

        //BNO055内のセンサーの参考温度を取得して実際の値に変換
        temp = (float)sensor1.getTemperature() * scTemp;

        //温湿度センサーより、温度、湿度、気圧、現在の状態を取得して変数に格納
        bme280_T = sensor2.getTemp();
        bme280_P = sensor2.getPress_hPa();
        bme280_H = sensor2.getHum();
        bme280_status = sensor2.getStatus();

/*
        //温湿度センサーの補正データが更新されていたなら、計算用数値を更新
        *if(sensor2.isReady()){
            sensor2.updateCalib();
        }

        printf(
            "Acc = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\nMag = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\nGyr = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\n",
            ax, ay, az, mx, my, mz, gx, gy, gz
        );

        printf("%lf %lf %lf\r\n",roll,pitch,yaw);

        printf(
            "Temperature\t = %03.3f[degC] (BNO055 -> %03.3f[degC])\r\nPressure\t = %06.3f[hPa]\r\nHumidity\t = %03.3f[%%RH]\r\nStatus\t = 0x%02X\r\n",
            bme280_T, temp, bme280_P, bme280_H, bme280_status
        );
*/
    }
}

void peakcheck(){
    double height = ((pow(1013.25/bme280_P , 1/5.257)-1)*(bme280_T+273.15))/0.0065;//外に出すとinfと表示される。原因は謎
    double n= height-previous_height;//相対高度 
    //printf("m=%d\r\nPressure=%f\r\nTemperature=%f\r\nHeight=%f[m]\r\nPrevious_height=%f\r\nn=%f\r\n",m,bme280_P,bme280_T,height,previous_height,n);
    //printf("m=f%d\r\npow(1013.25/bme280_p , 1/5.257)=%f\r\n(pow(1013.25/bme_280_P , 1/5.257)-1)*(bme280_T+273.15)=%f\r\n(pow(1013.25/bme_280_P , 1/5.257)-1)*(bme280_T+273.15)/0.0065=%f\r\nheight=%f\r\n",m,pow(1013.25/bme280_P , 1/5.257),(pow(1013.25/bme280_P , 1/5.257)-1)*(bme280_T+273.15),((pow(1013.25/bme280_P , 1/5.257)-1)*(bme280_T+273.15))/0.0065,height);
    
    printf("pressure=%f\r\n",bme280_P);
    printf("temperature=%f\r\n",bme280_T);
    printf("height=%f\r\n",height);
    //printf("previous_height=%f\r\n",previous_height);
    printf("n=%f\r\n",n);
    printf("m=%d\r\n",m);

    //↑previous_heightもprintfすると、値が変化しなくなる

    if(n<0){
        m++;
    }
    else{
        m=0;//カウントリセット
    }
    if(m==5){
        S2M = 1;//頂点検知をsubからmainへ送る
        peakchecktime.detach();
        printf("頂点検知成功\n");
    }
    previous_height = height;
}
