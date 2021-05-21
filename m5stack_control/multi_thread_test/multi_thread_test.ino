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
void RRespiratoryAssist();
void IRAM_ATTR usecTimer();
void task1();
void task2();

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
volatile unsigned long usecCount = 0;
hw_timer_t *interrupptTimer = NULL;
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
// 表示用
int display[2] = {0};
// タイマーのスタートストップチェック用
int countStart = 0;
int startCheck = 0;


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
    //timerBegin is count per 1 microsec.
    interrupptTimer = timerBegin(0, 80, true);
    //interrupt method setting
    timerAttachInterrupt(interrupptTimer, &usecTimer, true);
    //interrupt timing setting.
    timerAlarmWrite(interrupptTimer, 1000, true);
    timerAlarmDisable(interrupptTimer);
    /* ---------
    ディスプレイ設定
    --------- */
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setTextSize(4);
    M5.Lcd.print("SYSTEM\n ACTIVATE\n");

    /* --------------------
    制御開始までにLEDを3回点滅
    -------------------- */
    /*
    LED = 1;
    for(int i=0;i<6;i++){
        wait(0.5);
        LED = !LED;
    }*/
    delay(500);

    /* ----
    初期設定
    ---- */
    dacWrite(CONTROL, minCnt*255/3.3);
    delay(5000);                 // 最初は5秒間待機

    /* ----
    制御開始
    ---- */
    timerAlarmEnable(interrupptTimer);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setTextSize(4);
    M5.Lcd.print("WAITING\n");
}


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
        intime = 3.0;
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
        if((GoSign==1)&&(17<TriggerLPF[1])&&((usecCount / 1000)>1)){
            digitalWrite(TRIGGER, HIGH);
            digitalWrite(ASSIST, HIGH);
            M5.Lcd.setCursor(10, 10);
            M5.Lcd.clear(BLACK);
            M5.Lcd.setTextColor(RED, BLACK);
            M5.Lcd.setTextSize(3);
            M5.Lcd.print("V ASSIST\n");
            display[1] = (int)(usecCount / 1000);
            display[0] = (int)(usecCount / 60000);
            M5.Lcd.printf(" m: s: ms: us\n");
            M5.Lcd.printf("%02d:",display[0]);
            M5.Lcd.printf("%02d:\n",display[1]);
            RespiratoryAssist();
            digitalWrite(TRIGGER, LOW);
            digitalWrite(ASSIST, LOW);
            GoSign = 0;
        }
        // 自律制御
        if((usecCount/1000)>backup){
            digitalWrite(ASSIST, HIGH);
            M5.Lcd.setCursor(10, 10);
            M5.Lcd.clear(BLACK);
            M5.Lcd.setTextColor(RED, BLACK);
            M5.Lcd.setTextSize(4);
            M5.Lcd.print("A ASSIST\n");
            display[1] = (int)(usecCount / 1000);
            display[0] = (int)(usecCount / 60000);
            M5.Lcd.printf(" m: s\n");
            M5.Lcd.printf("%02d:",display[0]);
            M5.Lcd.printf("%02d:\n",display[1]);
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
    usecCount = 0;
    timerAlarmEnable(interrupptTimer);

    /* -----------
    制御周期カウンタ
    ----------- */
    float ControlCycleCount = 0;

    /* -------
    吸気支援開始
    ------- */
    while((usecCount/1000)<intime){
        // 制御周期を一定で実施
        if((usecCount / 1000)>ControlCycleCount){
            // D制御のための時間記憶
            t[1] = usecCount / 1000;  // 時間を記憶
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
    }

    /* -------------------
    各パラメタを待機設定に戻す
    ------------------- */
    // 待機設定に戻す
    e[0] = 0; e[1] = 0;
    t[0] = 0; t[1] = 0;
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setTextSize(3);
    M5.Lcd.print("WAITING\n");
    dacWrite(CONTROL, (minCnt/3.3)*255);
    usecCount = 0;
    timerAlarmEnable(interrupptTimer);
}

void IRAM_ATTR usecTimer(){
    portENTER_CRITICAL_ISR(&mutex);
    usecCount += 1;
    portEXIT_CRITICAL_ISR(&mutex);
}
