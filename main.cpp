#include "mbed.h"
#include "SDFileSystem.h"
#include "BNO055.h"
#include "BME280.h"
#include <cstdio>


Serial pc(USBTX, USBRX, 115200);

SDFileSystem sd(PA_7, PA_6, PA_5, PB_0, "sd");
DigitalIn sdcheck(D12);

I2C ifaceI2C(I2C_SDA, I2C_SCL);
BOARDC_BNO055 sensor1(&ifaceI2C);
BOARDC_BME280 sensor2(&ifaceI2C);


Timer t;

int main() {
    int sdmode =sdcheck;
    printf("SDmode= %d \r\n",sdmode); //0ならスロットイン
    mkdir("/sd/mydir100", 0777);
    FILE *fp = fopen("/sd/mydir100/sdtest.txt", "w");

    ifaceI2C.frequency(100000);
    sensor1.initialize(false);
    sensor2.initialize(false);
    //各センサーの値を格納するための変数宣言
    short dataBox[12];
    float scAcc, scMag, scGyro, scEUL, scTemp;
    float ax, ay, az, mx, my, mz, gx, gy, gz, temp;
    short L_accX, L_accY, L_accZ;
    double yaw, roll, pitch;
    float bme280_T = 0.0, bme280_P = 0.0, bme280_H = 0.0;
    char bme280_status = 0x00;
    //センサーのRAW値を実際の数値に変換するための倍率を取得する
    scAcc = sensor1.getAccScale();
    scMag = sensor1.getMagScale();
    scGyro = sensor1.getGyroScale();
    scEUL = sensor1.getEulerScale();
    scTemp = sensor1.getTempScale();

    t.start();
    int time;

    while(time < 10) {
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
        //温湿度センサーの補正データが更新されていたなら、計算用数値を更新
        if(sensor2.isReady()){
            sensor2.updateCalib();
        }
        //printf("Acc = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\nMag = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\n",ax, ay, az, mx, my, mz);

        //printf("pitch= %lf roll= %lf yaw= %lf\r\n",roll,pitch,yaw);

        //printf("Temperature\t = %03.3f[degC] (BNO055 -> %03.3f[degC])\r\nPressure\t = %06.3f[hPa]\r\nHumidity\t = %03.3f[%%RH]\r\nStatus\t = 0x%02X\r\n",bme280_T, temp, bme280_P, bme280_H, bme280_status);

        fprintf(fp,"Acc = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\nMag = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\n pitch= %lf roll= %lf yaw= %lf\r\nTemperature\t = %03.3f[degC] (BNO055 -> %03.3f[degC])\r\nPressure\t = %06.3f[hPa]\r\nHumidity\t = %03.3f[%%RH]\r\nStatus\t = 0x%02X\r\n",ax, ay, az, mx, my, mz,roll,pitch,yaw,bme280_T, temp, bme280_P, bme280_H, bme280_status);
        
        //線形加速度を取得//線形加速度は重力の影響が除去されており、デバイスの運動を知りたいときに見るらしい？
        sensor1.getLinearAccDataAll(L_accX, L_accY, L_accZ);

        fprintf(fp,"Acc = X[%06.5f], Y[%06.5f], Z[%06.5f]\r\n",L_accX, L_accY, L_accZ);

        time = t.read();
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
