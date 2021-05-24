#include<M5Stack.h>


/* ------
define
------ */
// PD制御ゲイン
#define kpV 0.20           // VCV用 ***
#define kdV 0.0025         // VCV用 ***
#define kpP 0.30           // PCV用 ***
#define kdP 0.008          // PCV用 ***
// モータ制御電圧の上限・下限
#define minCnt 0.505       // 遠心ポンプのドライバが駆動する制御電圧の下限
#define maxCnt 3.205       // 回路的に念のために設ける上限
// 制御周期
#define ControlCycle 0.002  // 50Hzの制御周期
//　m5stackのボタンピン番号
#define BUTTONA 39
#define BUTTONB 38
#define BUTTONC 37
// 各機能のピン番号
#define CONTROL 26    // 制御入力
#define FLOW 35        // 流量センサ
#define PRESSURE 36    // 圧力センサ
#define ASSIST 1    // 支援信号
#define TRIGGER 17   // トリガ信号
// ディスプレイ用
#define LCDHIGH 240
#define LCDWIDTH 320

/* -------------
制御パラメタ設定
------------- */
int TARGET = 1;      // 流量or圧力調整 ***
int INTIME = 0;      // 吸気時間調整
int MODE = 0;        // モード切替[0:VCV][1:PCV] ***

/* ----------------
関数のプロトタイプ宣言
---------------- */
void RespiratoryAssist();

/* ----------------------------------------
吸気支援関数内で主に扱う変数：global変数として宣言
---------------------------------------- */
// ボリューム用パラメタ
float target;              // 流量or圧力対応 ***
float intime;              // 吸気時間対応
// PD制御用パラメタ
float raw;                 // 流量or圧力センサの生値 ***
float e[2];                // 制御偏差
float t[2];                // 時間
float dt;                  // D制御に用いる時間差分
float checkCONTROL;        // 制御入力の[min-max]包含チェック
float preCONTROL;          // 前回の制御入力
float p,d;                 // PD制御
// 随意トリガ用パラメタ
float TriggerRaw;          // 流量センサの生値を物理量に変換
float TriggerLPF[2];       // 1次のデジタルLPFを施行
// プログラム終了トリガ（仮）
float preMODE = MODE;
int EndTrigger = 0;
// タイマーの変数
unsigned long start, end, diff;
// 表示用
int display[2] = {0};


void setup() {
    M5.begin();
    /* ----
    ピン設定
    ---- */
    pinMode(BUTTONA, INPUT);
    pinMode(BUTTONB, INPUT);
    pinMode(BUTTONC, INPUT);
    pinMode(ASSIST, OUTPUT);
    pinMode(TRIGGER, OUTPUT);
    /* ------
    タイマー設定
    ------ */

    /* ---------
    ディスプレイ設定
    --------- */
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.clear(BLACK);
    //M5.Lcd.setTextColor(RED, BLACK);
    //M5.Lcd.setTextSize(4);
    //M5.Lcd.print("SYSTEM\n ACTIVATE\n");
    //M5.Lcd.clear(BLACK);

    /* --------------------
    制御開始までにLEDを3回点滅
    -------------------- */
    /*
    LED = 1;
    for(int i=0;i<6;i++){
        wait(0.5);
        LED = !LED;
    }*/
    //delay(500);
    

    /* ----
    初期設定
    ---- */
    int temp = (int)(minCnt*255/3.3);
    dacWrite(CONTROL, temp);
    delay(5000);                 // 最初は5秒間待機
    start = millis() / 1000;


    /* ----
    制御開始
    ---- */
    //M5.Lcd.setCursor(10, 10);
    //M5.Lcd.clear(BLACK);
    //M5.Lcd.setTextColor(RED, BLACK);
    //M5.Lcd.setTextSize(4);
    //M5.Lcd.print("WAITING\n");
}
/*
void loop(){
    //Serial.println("reset\n");
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setTextSize(4);
    M5.Lcd.print("SYSTEM\n ACTIVATE\n");
    
    start = millis() / 2000;
    end = start;
    //Serial.print(start-end);
    diff = start - end;
    //int temp = INTIME * 1000;
    while(diff < INTIME){
        M5.Lcd.clear(BLACK);
        //Serial.print(start - end);
        end = millis() / 1000;
        diff = start - end;
    }
}
*/
void loop() {
    /* -------------------------------------
    main関数内でのみ扱う変数：local変数として宣言
    ------------------------------------- */
    // 自律トリガ用パラメタ
    float backup = 0;
    // 随意トリガ用パラメタ
    int GoSign = 0;          // 定常状態にまで落ちてきたらON

    while(EndTrigger < 8){
        /* --------------
        自律トリガ用パラメタ
        -------------- */
        // 吸気時間の調整
        intime = 2.0;
        //intime = INTIME+2;   // 正確には(3.3*INTIME)*(2/3.3)+2
        // バックアップ換気の間隔
        backup = intime*2;     // I:E比の比率だけ乗算

        /* --------------
        随意トリガ用パラメタ
        -------------- */
        // 流量センサ値の物理量化と平滑化
        TriggerRaw = 37.5*(analogRead(FLOW)/4096)*3.3 - 68.75;
        TriggerLPF[1] = 0.8*TriggerLPF[0] + 0.2*TriggerRaw;
        // GoSignの更新
        if((GoSign==0)&&(TriggerLPF[1]<11)){
            GoSign = 1;
        }

        /* ----
        吸気支援
        ---- */
        // 随意制御
        end = millis() / 1000;
        diff = end - start;
        /*
        if((GoSign==1)&&(17<TriggerLPF[1])&&(diff>1)){
            digitalWrite(TRIGGER, HIGH);
            digitalWrite(ASSIST, HIGH);
            //M5.Lcd.setCursor(10, 10);
            //M5.Lcd.clear(BLACK);
            //M5.Lcd.setTextColor(RED, BLACK);
            //M5.Lcd.setTextSize(3);
            //M5.Lcd.print("V ASSIST\n");
            //display[1] = (int)(usecCount / 1000);
            //display[0] = (int)(usecCount / 60000);
            //M5.Lcd.printf(" m: s: ms: us\n");
            //M5.Lcd.printf("%02d:",display[0]);
            //M5.Lcd.printf("%02d:\n",display[1]);
            RespiratoryAssist();
            digitalWrite(TRIGGER, LOW);
            digitalWrite(ASSIST, LOW);
            GoSign = 0;
        }*/
        // 自律制御
        if(diff>backup){
            digitalWrite(ASSIST, HIGH);
            //M5.Lcd.setCursor(10, 10);
            //M5.Lcd.clear(BLACK);
            //M5.Lcd.setTextColor(RED, BLACK);
            //M5.Lcd.setTextSize(4);
            //M5.Lcd.print("A ASSIST\n");
            //display[1] = (int)(usecCount / 1000);
            //display[0] = (int)(usecCount / 60000);
            //M5.Lcd.printf(" m: s\n");
            //M5.Lcd.printf("%02d:",display[0]);
            //M5.Lcd.printf("%02d:\n",display[1]);
            RespiratoryAssist();
            digitalWrite(ASSIST, LOW);
            GoSign = 0;
        }
        
        // 随意トリガ用パラメタの前回値を記憶
        TriggerLPF[0] = TriggerLPF[1];
        if(abs(MODE-preMODE)>0.9){
            EndTrigger++;
        }
        preMODE = MODE;
    }
}


/* ---------------
... 吸気支援関数 ...
--------------- */
void RespiratoryAssist(){
    /* -----------
    流量or圧力の調整
    ----------- */
    if(MODE<0.5){
        target = TARGET*1.2 + 2.1;                                   // 正確には(3.3*TARGET)*(0.800/3.3)+2.5 ***
    }
    else{
        target = TARGET*1.911 + 1.389;                                 // 正確には(3.3*TARGET)*(1.911/3.3)+1.389 ***
    }

    /* ----------------
    吸気時間用タイマー起動
    ---------------- */
    start = millis() / 1000;

    /* -----------
    制御周期カウンタ
    ----------- */
    float ControlCycleCount = 0;

    /* -------
    吸気支援開始
    ------- */
    end = start;
    diff = end - start;
    while(diff<=intime){
        // 制御周期を一定で実施
        if(diff>=ControlCycleCount){
            // D制御のための時間記憶
            end = millis() / 1000;
            t[1] = end - start;  // 時間を記憶
            // 自発呼吸トリガ用パラメタ
            TriggerRaw = 37.5*(analogRead(FLOW)/4096)*3.3 - 68.75;
            TriggerLPF[1] = 0.8*TriggerLPF[0] + 0.2*TriggerRaw;
            // P:比例制御 ***
            if(MODE<0.5){
                raw = (analogRead(FLOW)/4096)*3.3;
            }else{
                raw = (analogRead(PRESSURE)/4096)*3.3;
            }
            e[1] = target - raw;                                         // 偏差を記憶 ***
            p = (ControlCycleCount==0) ? 0 : e[1];                       // 原始ループでのP制御は不実施
            // D:微分制御
            dt = t[1] - t[0];
            d = (ControlCycleCount==0) ? 0 : (e[1]-e[0])/dt;             // 原始ループでのD制御は不実施
            // PD:比例微分制御 ***
            preCONTROL = (ControlCycleCount==0) ? minCnt*2 : preCONTROL; // 原始ループでのpreCONTROLはminCnt*2に設定
            if(MODE<0.5){
                checkCONTROL = preCONTROL + kpV*p + kdV*d;                 // PD制御 ***
            }else{
                checkCONTROL = preCONTROL + kpP*p + kdP*d;
            }
            // [min-max]の範囲内に入っているかの確認
            if((minCnt<checkCONTROL)&&(checkCONTROL<maxCnt)){
                dacWrite(CONTROL, preCONTROL=(checkCONTROL/3.3)*255);
            }else if(checkCONTROL<minCnt){
                dacWrite(CONTROL, preCONTROL=(minCnt/3.3)*255);
            }else if(maxCnt<checkCONTROL){
                dacWrite(CONTROL, preCONTROL=(maxCnt/3.3)*255);
            }
            // 各パラメタの前回値を記憶
            e[0] = e[1];
            TriggerLPF[0] = TriggerLPF[1];
            t[0] = t[1];
            // 制御周期カウンタのインクリメント
            ControlCycleCount += ControlCycle;
        }
        end = millis() / 1000;
        diff = end - start;
    }

    /* -------------------
    各パラメタを待機設定に戻す
    ------------------- */
    // 待機設定に戻す
    e[0] = 0; e[1] = 0;
    t[0] = 0; t[1] = 0;
    //M5.Lcd.setCursor(10, 10);
    //M5.Lcd.clear(BLACK);
    //M5.Lcd.setTextColor(RED, BLACK);
    //M5.Lcd.setTextSize(3);
    //M5.Lcd.print("WAITING\n");
    dacWrite(CONTROL, (minCnt/3.3)*255);
    start = millis() / 1000;
}
