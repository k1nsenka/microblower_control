#include "mbed.h"

// PD制御ゲイン
#define kpV 0.20           // VCV用 ***
#define kdV 0.0025         // VCV用 ***
#define kpP 0.30           // PCV用 ***
#define kdP 0.008          // PCV用 ***
// モータ制御電圧の上限・下限
#define minCnt 0.505       // 遠心ポンプのドライバが駆動する制御電圧の下限
#define maxCnt 3.205       // 回路的に念のために設ける上限
// 制御周期
#define ControlCycle 0.02  // 50Hzの制御周期

/* -------------
mbed側パラメタ設定
------------- */
Timer TIMER;               // タイマー
AnalogIn TARGET(p15);      // 流量or圧力調整 ***
AnalogIn INTIME(p16);      // 吸気時間調整
AnalogIn MODE(p17);        // モード切替[0:VCV][1:PCV] ***
AnalogOut CONTROL(p18);    // 制御入力
AnalogIn FLOW(p19);        // 流量センサ
AnalogIn PRESSURE(p20);    // 圧力センサ
DigitalOut LED(LED1);      // LED
DigitalOut ASSIST(p21);    // 支援信号
DigitalOut TRIGGER(p22);   // トリガ信号

/* ----------------
関数のプロトタイプ宣言
---------------- */

void RespiratoryAssist();  // 吸気支援関数

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


int main(){
    /* -------------------------------------
    main関数内でのみ扱う変数：local変数として宣言
    ------------------------------------- */
    // 自律トリガ用パラメタ
    float backup = 0;
    // 随意トリガ用パラメタ
    int GoSign = 0;          // 定常状態にまで落ちてきたらON

    /* --------------------
    制御開始までにLEDを3回点滅
    -------------------- */
    LED = 1;
    for(int i=0;i<6;i++){
        wait(0.5);
        LED = !LED;
    }

    /* ----
    初期設定
    ---- */
    CONTROL = minCnt/3.3;
    wait(5);                 // 最初は5秒間待機
    MODE = 0

    /* ----
    制御開始
    ---- */
    TIMER.start();
    while(EndTrigger<5){
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
        TriggerRaw = 37.5*(3.3*FLOW) - 68.75;
        TriggerLPF[1] = 0.8*TriggerLPF[0] + 0.2*TriggerRaw;
        // GoSignの更新
        if((GoSign==0)&&(TriggerLPF[1]<11)){
            GoSign = 1;
        }

        /* ----
        吸気支援
        ---- */
        // 随意制御
        if((GoSign==1)&&(14<TriggerLPF[1])&&(TIMER.read()>1)){
        //&&(TriggerLPF[1]<18)){
            TRIGGER = 1;
            ASSIST = 1;
            RespiratoryAssist();
            TRIGGER = 0;
            ASSIST = 0;
            GoSign = 0;
        }
        // 自律制御
        if(TIMER.read()>backup){
            ASSIST = 1;
            RespiratoryAssist();
            ASSIST = 0;
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
    TARGET = 1.0
    if(MODE<0.5){
        target = TARGET*1.2 + 2.1;                                   // 正確には(3.3*TARGET)*(0.800/3.3)+2.5 ***
    }
    else{
        target = TARGET*1.911 + 1.389;                                 // 正確には(3.3*TARGET)*(1.911/3.3)+1.389 ***
    }

    /* ----------------
    吸気時間用タイマー起動
    ---------------- */
    TIMER.reset();
    TIMER.start();

    /* -----------
    制御周期カウンタ
    ----------- */
    float ControlCycleCount = 0;

    /* -------
    吸気支援開始
    ------- */
    while(TIMER.read()<intime){
        // 制御周期を一定で実施
        if(TIMER.read()>ControlCycleCount){
            // D制御のための時間記憶
            t[1] = TIMER.read();                                         // 時間を記憶
            // 自発呼吸トリガ用パラメタ
            TriggerRaw = 37.5*(3.3*FLOW) - 68.75;
            TriggerLPF[1] = 0.8*TriggerLPF[0] + 0.2*TriggerRaw;
            // P:比例制御 ***
            if(MODE<0.5){
                raw = 3.3*FLOW;
            }else{
                raw = 3.3*PRESSURE;
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
                CONTROL = checkCONTROL/3.3;
            }else if(checkCONTROL<minCnt){
                CONTROL = minCnt/3.3;
            }else if(maxCnt<checkCONTROL){
                CONTROL = maxCnt/3.3;
            }
            // 各パラメタの前回値を記憶
            preCONTROL = CONTROL*3.3;
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
    CONTROL = minCnt/3.3;
    TIMER.reset();
    TIMER.start();
}
