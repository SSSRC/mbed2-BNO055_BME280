#include "mbed.h"
#include "SDFileSystem.h"
#include "BNO055.h"
#include "BME280.h"
#include <cstdio>

Ticker peakchecktime;

void peakcheck();

Serial pc(USBTX, USBRX, 9600);

//mbed LED1
DigitalOut led(LED1);

SDFileSystem sd(PA_7, PA_6, PA_5, PB_0, "sd");
DigitalIn sdcheck(D12);

I2C ifaceI2C(I2C_SDA, I2C_SCL);
BOARDC_BNO055 sensor1(&ifaceI2C);
BOARDC_BME280 sensor2(&ifaceI2C);

DigitalIn M2S(PA_8);
DigitalOut S2M(PA_11);

Timer t;

double bme280_P = 0.0;
double bme280_T = 0.0;
//double height = ((pow(1013.25/bme280_P , 1/5.257)-1)*(bme280_T+273.15))/0.0065;
double previous_height = 0.0;
//double n = height-previous_height;//相対高度 
int m = 0;//相対高度が連続して負になった回数

int main() {
    while(1){
        if(M2S == 1){
                int sdmode =sdcheck;
                printf("SDmode= %d \r\n",sdmode); //0ならスロットイン
                mkdir("/sd/mydir100", 0777);
                FILE *fp = fopen("/sd/mydir100/sdtest.txt", "w");

                ifaceI2C.frequency(100000);
                sensor1.initialize(false);
                sensor2.initialize(false);
                //各センサーの値を格納するための変数宣言
                short dataBox[12];
                float scAcc, scMag, scGyro, scEUL, scTemp, scLIA; //scLIA追加
                float ax, ay, az, mx, my, mz, gx, gy, gz, temp;
                short L_accX, L_accY, L_accZ;
                double yaw, roll, pitch;
                float /*bme280_T = 0.0, bme280_P = 0.0,*/ bme280_H = 0.0;
                char bme280_status = 0x00;
                //センサーのRAW値を実際の数値に変換するための倍率を取得する
                scAcc = sensor1.getAccScale();
                scMag = sensor1.getMagScale();
                scGyro = sensor1.getGyroScale();
                scEUL = sensor1.getEulerScale();
                scTemp = sensor1.getTempScale();
                scLIA = sensor1.getLinearScale(); 

                t.start();
                int time;
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
                    /*
                    yaw = (float)dataBox[9] * scEUL;
                    roll = (float)dataBox[10] * scEUL;
                    pitch = (float)dataBox[11] * scEUL;
                    */
                    sensor1.getEulerFromQ(yaw,roll,pitch);
                    //BNO055内のセンサーの参考温度を取得して実際の値に変換
                    temp = (float)sensor1.getTemperature() * scTemp;
                    //温湿度センサーより、温度、湿度、気圧、現在の状態を取得して変数に格納
                    bme280_T = sensor2.getTemp();
                    bme280_P = sensor2.getPress_hPa();
                    bme280_H = sensor2.getHum();
                    bme280_status = sensor2.getStatus();
                    //温湿度センサーの補正データが更新されていたなら、計算用数値を更新
                    if(sensor2.isReady()){
                        sensor2.updateCalib();
                    }
                    //printf("Acc = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\nMag = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\n",ax, ay, az, mx, my, mz);
                
                    printf("pitch= %lf roll= %lf yaw= %lf\r\n",roll,pitch,yaw);//yawの所がrollになっている
                    fprintf(fp,"pitch= %lf roll= %lf yaw= %lf\r\n",roll,pitch,yaw);
                
                    //printf("Temperature\t = %03.3f[degC] (BNO055 -> %03.3f[degC])\r\nPressure\t = %06.3f[hPa]\r\nHumidity\t = %03.3f[%%RH]\r\nStatus\t = 0x%02X\r\n",bme280_T, temp, bme280_P, bme280_H, bme280_status);

                    //fprintf(fp,"Acc = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\nMag = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\n pitch= %lf roll= %lf yaw= %lf\r\nTemperature\t = %03.3f[degC] (BNO055 -> %03.3f[degC])\r\nPressure\t = %06.3f[hPa]\r\nHumidity\t = %03.3f[%%RH]\r\nStatus\t = 0x%02X\r\n",ax, ay, az, mx, my, mz,roll,pitch,yaw,bme280_T, temp, bme280_P, bme280_H, bme280_status);
                    
                    //線形加速度を取得//線形加速度は重力の影響が除去されており、デバイスの運動を知りたいときに見るらしい？
                    
                    /*
                    short a =sensor1.getLinearAccDataX();
                    short b =sensor1.getLinearAccDataY();
                    short c =sensor1.getLinearAccDataZ();
                    printf("b=%f, a=%f, c=%f\r\n",b,a,c);
                    */
                
                    //sensor1.getLinearAccDataAll(L_accX, L_accY, L_accZ)
                    L_accX = sensor1.getLinearAccDataX() * scLIA;
                    L_accY = sensor1.getLinearAccDataY() * scLIA;
                    L_accZ = sensor1.getLinearAccDataZ() * scLIA;
                
                    printf("Acc = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\n", L_accX, L_accY, L_accZ);
                    fprintf(fp,"Acc = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\n",L_accX, L_accY, L_accZ);
            
                    time = t.read();

                    if(M2S == 0){
                        break;
                    }
                }
                fclose(fp); 
                
                //.txtファイル読み込み
                fp = fopen("/sd/mydir100/sdtest.txt", "r");
                if(fp == NULL) {
                    pc.printf("Could not open file for write\n");
                }
                // バッファサイズの定義
                char buffer[128];
                while (fgets(buffer, sizeof(buffer), fp) != NULL) {
                    pc.printf("%s", buffer);
                }
                fclose(fp); 
        }  
    }
}


void peakcheck(){
    double height = ((pow(1013.25/bme280_P , 1/5.257)-1)*(bme280_T+273.15))/0.0065;//外に出すとinfと表示される。原因は謎
    double n=height-previous_height;//相対高度 
    //printf("m=%d\r\nPressure=%f\r\nTemperature=%f\r\nHeight=%f[m]\r\nPrevious_height=%f\r\nn=%f\r\n",m,bme280_P,bme280_T,height,previous_height,n);
    //printf("m=f%d\r\npow(1013.25/bme280_p , 1/5.257)=%f\r\n(pow(1013.25/bme_280_P , 1/5.257)-1)*(bme280_T+273.15)=%f\r\n(pow(1013.25/bme_280_P , 1/5.257)-1)*(bme280_T+273.15)/0.0065=%f\r\nheight=%f\r\n",m,pow(1013.25/bme280_P , 1/5.257),(pow(1013.25/bme280_P , 1/5.257)-1)*(bme280_T+273.15),((pow(1013.25/bme280_P , 1/5.257)-1)*(bme280_T+273.15))/0.0065,height);
    
   /* printf("pressure=%f\r\n",bme280_P);
    printf("temperature=%f\r\n",bme280_T);
    printf("height=%f\r\n",height);
    printf("n=%f\r\n",n);
    printf("m=%d\r\n",m);
    */

    if(n<-0.1){
        m++;
    }
    else{
        m=0;//カウントリセット
    }

    printf("%f,%f,%d\r\n",bme280_P,t.read(),m);

    if(m==5){
        S2M = 1;//頂点検知をsubからmainへ送る
        peakchecktime.detach();
        printf("頂点検知成功\n");
    }
    previous_height = height;
}