#include<M5Stack.h>
#include<M5TreeView.h>

/* ------
define
------ */
// PD制御ゲイン
#define kpV 0.50           // VCV用 ***
#define kdV 0.012         // VCV用 ***
#define kpP 0.50           // PCV用 ***
#define kdP 0.012          // PCV用 ***
// モータ制御電圧の上限・下限
#define minCnt 39.02       // 遠心ポンプのドライバが駆動する制御電圧の下限
#define maxCnt 247.66    // 回路的に念のために設ける上限
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
#define ASSIST 0    // 支援信号
#define TRIGGER 17   // トリガ信号
// ディスプレイ用
#define LCDHIGH 240
#define LCDWIDTH 320

/* ------
画面描画用クラス
------ */
M5TreeView treeView;
typedef std::vector<MenuItem*> vmi;

/* -------------
制御パラメタ設定
二つのタスクで共通化
------------- */
volatile float S_TARGET = 0.5;      // 流量or圧力調整 ***
volatile float S_INTIME = 0;      // 吸気時間調整
volatile float S_MODE = 0;        // モード切替[0:VCV][1:PCV] ***
volatile float S_TORIGGER = 14;  //閾値の設定

/* -----------
データロガー用変数
------------ */
File file;
unsigned int log_start = 0;
unsigned int log_end = 0;
unsigned int log_diff = 0;
const char* fname = "/data_log.csv";
float log_flow_sensor;
float log_flow_v;
float log_flow;
float log_pressure_sensor;
float log_pressure_v;
float log_pressure;
float log_flow_LPF[2];
float log_pressure_LPF[2];
int log_ASSIST;
int log_TRIGGER;

/* ----------------------------------------
吸気支援関数内で主に扱う変数：global変数として宣言
---------------------------------------- */
// ボリューム用パラメタ
float target = 2.1;              // 流量or圧力対応 ***
float intime;              // 吸気時間対応
// PD制御用パラメタ
float raw;                 // 流量or圧力センサの生値 ***
float e[2];                // 制御偏差
float t[2];                // 時間
float preTime;
float dt;                  // D制御に用いる時間差分
float checkCONTROL = 0;        // 制御入力の[min-max]包含チェック
float preCONTROL;          // 前回の制御入力
float temp_cnt;
float p,d;                 // PD制御
// 随意トリガ用パラメタ
float TriggerRaw;          // 流量センサの生値を物理量に変換
float TriggerLPF[2];       // 1次のデジタルLPFを施行
float flow_sensor;         //センサ値の取得用
float press_sensor;        //センサ値の取得用
// プログラム終了トリガ（仮）
float preMODE = S_MODE;
int EndTrigger = 0;
// タイマーの変数
unsigned long start, end, diff;

/* ----------------
関数のプロトタイプ宣言
---------------- */
void RespiratoryAssist();
void mode_setting(MenuItem* sender);
void intime_setting(MenuItem* sender);
void target_setting(MenuItem* sender);
void torigger_setting(MenuItem* sender);
void display_task(void *arg);
void control_task(void *arg);

/* ---------------
... 吸気支援関数 ...
--------------- */
void RespiratoryAssist(){
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
    while(1){
        /* -----------
        流量or圧力の調整
        ----------- */
        if(S_MODE<0.5){
            target = S_TARGET*1.2 + 2.1;               // 正確には(3.3*TARGET)*(0.800/3.3)+2.5 ***
        }
        else{
            target = S_TARGET*1.911 + 1.389;          // 正確には(3.3*TARGET)*(1.911/3.3)+1.389 ***
        }
        //preTime = millis();
        // 制御周期を一定で実施
        // D制御のための時間記憶
        // 自発呼吸トリガ用パラメタ
        flow_sensor = analogRead(FLOW);
        TriggerRaw = 37.5*((flow_sensor+2143)/4096)*3.3 - 68.75;
        TriggerLPF[1] = 0.8*TriggerLPF[0] + 0.2*TriggerRaw;
        // P:比例制御 ***
        if(S_MODE<0.5){
            raw = ((flow_sensor+2143)/4096)*3.3;
        }else{
            press_sensor = analogRead(PRESSURE);
            raw = (press_sensor/4096)*3.3;
        }
        e[1] = target - raw;                                         // 偏差を記憶 ***
        p = (ControlCycleCount==0) ? 0 : e[1];                       // 原始ループでのP制御は不実施
        // D:微分制御
        //dt =  (millis() - preTime);
        d = (ControlCycleCount==0) ? 0 : (e[1]-e[0]);             // 原始ループでのD制御は不実施
        // PD:比例微分制御 ***
        preCONTROL = (ControlCycleCount==0) ? minCnt : preCONTROL; // 原始ループでのpreCONTROLはminCnt*2に設定
        if(S_MODE == 0){
            checkCONTROL = preCONTROL + kpV*p + kdV*d;                 // PD制御 ***
        }else{
            checkCONTROL = preCONTROL + kpP*p + kdP*d;
        }
        // [min-max]の範囲内に入っているかの確認
        if((minCnt<=checkCONTROL)&&(checkCONTROL<=maxCnt)){
            temp_cnt = checkCONTROL;
        }else if(checkCONTROL<minCnt){
            temp_cnt = minCnt;
        }else if(maxCnt<checkCONTROL){
            temp_cnt = maxCnt;
        }
        /*
        Serial.print(preCONTROL);
        Serial.print("#");
        Serial.print(checkCONTROL);
        Serial.print("#");
        Serial.print(temp_cnt);
        Serial.println("");
        */
        dacWrite(CONTROL, temp_cnt);
        preCONTROL = temp_cnt;
        // 各パラメタの前回値を記憶
        e[0] = e[1];
        TriggerLPF[0] = TriggerLPF[1];
        // 制御周期カウンタのインクリメント
        ControlCycleCount += ControlCycle;
        end = millis() / 1000;
        diff = end - start;
        delay(1);
    }
    /* -------------------
    各パラメタを待機設定に戻す
    ------------------- */
    // 待機設定に戻す
    e[0] = 0; e[1] = 0;
    dacWrite(CONTROL, minCnt);
    start = millis() / 1000;
}

/* --------------------
... マルチタスクsetup ...
-------------------- */
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
    タスクハンドラ設定
    ------ */
    TaskHandle_t t0_h;
    TaskHandle_t t1_h;
    /* ----
    タスク割り当て
    ---- */
    xTaskCreatePinnedToCore(display_task, "display_task", 4096, NULL, 1, &t0_h, 0);
    xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, 10, &t1_h, 1);
}

void loop(){}

void mode_setting(MenuItem* sender){
    switch(sender -> tag){
        default: return;
        case 111:
            //Serial.println("############################");
            S_MODE = 0;
            //Serial.println("case flow mode");
            break;
        case 112:
            //Serial.println("############################");
            S_MODE = 1;
            //Serial.println("case pressure mode");
            break;
    }
    M5.Lcd.fillRect(0, 218, M5.Lcd.width(), 22, 0);
}

void intime_setting(MenuItem* sender){
    switch(sender -> tag){
        default: return;
        case 121:
            //Serial.println("############################");
            if((0.0 <= S_INTIME)&&(S_INTIME < 1.0)){
                S_INTIME = S_INTIME + 0.1;
                //Serial.println("case intime up");
            }else if(S_INTIME < 0.0){
                S_INTIME = 0.0;
                //Serial.println("case intime min");
            }else if(1.0 <= S_INTIME){
                S_INTIME = 3.0;
                //Serial.println("case intime max");
            }
            break;
        case 122:
            //Serial.println("############################");
            if((0.0 < S_INTIME)&&(S_INTIME <= 1.0)){
                S_INTIME = S_INTIME - 0.1;
                //Serial.println("case intime down");
            }else if(S_INTIME <= 0.0){
                S_INTIME = 0.0;
                //Serial.println("case intime min");
            }else if(1.0 < S_INTIME){
                S_INTIME = 1.0;
                //Serial.println("case intime max");
            }
            break;
    }
    M5.Lcd.fillRect(0, 218, M5.Lcd.width(), 22, 0);
}

void target_setting(MenuItem* sender){
    switch(sender -> tag){
        default: return;
        case 131:
            //Serial.println("############################");
            if((0.0 <= S_TARGET)&&(S_TARGET < 1.0)){
                S_TARGET = S_TARGET + 0.1;
                //Serial.println("case target up");
            }else if(S_TARGET < 0.0){
                S_TARGET = 0.0;
                //Serial.println("case target min");
            }else if(1.0 <= S_TARGET){
                S_TARGET = 1.0;
                //Serial.println("case target max");
            }
            break;
        case 132:
            //Serial.println("############################");
            if((0.0 < S_TARGET)&&(S_TARGET <= 1.0)){
                S_TARGET = S_TARGET - 0.1;
                //Serial.println("case target down");
            }else if(S_TARGET <= 0.0){
                S_TARGET = 0.0;
                //Serial.println("case target min");
            }else if(1.0 < S_TARGET){
                S_TARGET = 1.0;
                //Serial.println("case target max");
            }
            break;
    }
    M5.Lcd.fillRect(0, 218, M5.Lcd.width(), 22, 0);
}

void torigger_setting(MenuItem* sender){
    switch(sender -> tag){
        default: return;
        case 131:
            //Serial.println("############################");
            if((10.0 <= S_TORIGGER)&&(S_TORIGGER < 20.0)){
                S_TORIGGER = S_TORIGGER + 1;
                //Serial.println("case torigger up");
            }else if(S_TORIGGER < 10.0){
                S_TORIGGER = 10.0;
                //Serial.println("case torigger min");
            }else if(20.0 <= S_TORIGGER){
                S_TORIGGER = 20.0;
                //Serial.println("case torigger max");
            }
            break;
        case 132:
            //Serial.println("############################");
            if((10.0 < S_TORIGGER)&&(S_TORIGGER <= 20.0)){
                S_TORIGGER = S_TORIGGER - 1;
                //Serial.println("case torigger down");
            }else if(S_TORIGGER <= 10.0){
                S_TORIGGER = 10.0;
                //Serial.println("case torigger min");
            }else if(20.0 < S_TORIGGER){
                S_TORIGGER = 20.0;
                //Serial.println("case torigger max");
            }
            break;
    }
    M5.Lcd.fillRect(0, 218, M5.Lcd.width(), 22, 0);
}

/* ---------------
... 画面描画タスク ...
--------------- */
void display_task(void *arg){
    Wire.begin();

    treeView.useFACES       = true;
    treeView.useCardKB      = true;
    treeView.useJoyStick    = true;
    treeView.usePLUSEncoder = true;
    treeView.useFACESEncoder= true;
    treeView.clientRect.x = 2;
    treeView.clientRect.y = 10;
    treeView.clientRect.w = 316;
    treeView.clientRect.h = 216;

    treeView.setItems(vmi
                    { new MenuItem("SETTING", 1, vmi
                        { new MenuItem("MODE", mode_setting, vmi
                            { new MenuItem("FLOW CONTROL", 111)
                            , new MenuItem("PRESSURE CONTROL", 112)
                            } )
                        , new MenuItem("INTIME", intime_setting, vmi
                            { new MenuItem("INTIME UP", 121)
                            , new MenuItem("INTIME DOWN", 122)
                            } )
                        , new MenuItem("TARGET", target_setting, vmi
                            { new MenuItem("TARGET UP", 131)
                            , new MenuItem("TARGET DOWN", 132)
                            } )
                        , new MenuItem("TORIGGER", torigger_setting, vmi
                            { new MenuItem("TORIGGER UP", 131)
                            , new MenuItem("TORIGGER DOWN", 132)
                            } )
                        } )
                    }
                );
    treeView.begin();
    log_start = millis();
    while(1){
        MenuItem* mi = treeView.update();
        if (mi != NULL) {
            M5.Lcd.fillRect(0,0,320,8,0);
            M5.Lcd.setTextColor(0xffff,0);
            M5.Lcd.setTextSize(1);
            M5.Lcd.drawString("menu:" + mi->title + " / tag:" + mi->tag, 15, 0, 1);
        }
        log_end = millis();
        log_diff += log_end - log_start;
        //流量計算
        log_flow_sensor = analogRead(FLOW);
        log_flow_v = ((log_flow_sensor+2143)/4096)*3.3;
        log_flow = 37.5*log_flow_v - 68.75;
        log_flow_LPF[1] = 0.8*log_flow_LPF[0] + 0.2*log_flow;
        //圧力計算
        log_pressure_sensor = analogRead(PRESSURE);
        log_pressure_v = (log_pressure_sensor/4096)*3.3;
        log_pressure = 11.249*log_pressure_v - 10.622;
        log_pressure_LPF[1] = 0.8*log_pressure_LPF[0] + 0.2*log_pressure;
        //ASSIST, TRIGGER
        log_ASSIST = digitalRead(ASSIST);
        log_TRIGGER = digitalRead(TRIGGER);
        //データログ
        file = SD.open(fname, FILE_APPEND);
        file.println(
            (String)log_diff + "," + 
            (String)p + "," + 
            (String)d + "," + 
            (String)target + "," + 
            (String)log_pressure_v + "," +
            (String)log_flow_v + "," + 
            (String)S_INTIME + "," + 
            (String)S_TORIGGER + "," + 
            (String)S_MODE + "," + 
            (String)log_flow_sensor + "," + 
            (String)log_flow + "," + 
            (String)log_flow_LPF[1] + "," + 
            (String)log_pressure + "," + 
            (String)log_pressure_LPF[1] + "," +
            (String)log_ASSIST + "," +
            (String)log_TRIGGER
        );
        file.close();
        log_start = log_end;
        log_flow_LPF[0] = log_flow_LPF[1];
        log_pressure_LPF[0] = log_pressure_LPF[1];
        delay(1);
    }
}

/* ---------------
... 制御タスク ...
--------------- */
void control_task(void *arg){
    /* ----
    初期設定
    ---- */
    dacWrite(CONTROL, 0);
    delay(5000);                 // 最初は5秒間待機
    /* ----
    制御開始
    ---- */
    start = millis() / 1000;
    while(1){
        // 自律トリガ用パラメタ
        float backup = 0;
        // 随意トリガ用パラメタ
        int GoSign = 0;          // 定常状態にまで落ちてきたらON
        while(EndTrigger < 10){
            /* --------------
            自律トリガ用パラメタ
            -------------- */
            // 吸気時間の調整
            intime = S_INTIME+2;   // 正確には(3.3*INTIME)*(2/3.3)+2
            // バックアップ換気の間隔
            backup = intime*2;     // I:E比の比率だけ乗算
            /* --------------
            随意トリガ用パラメタ
            -------------- */
            // 流量センサ値の物理量化と平滑化
            flow_sensor = analogRead(FLOW);
            TriggerRaw = 37.5*((flow_sensor+2143)/4096)*3.3 - 68.75;
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
            // 自律制御
            if(diff>backup){
                digitalWrite(ASSIST, HIGH);
                RespiratoryAssist();
                digitalWrite(ASSIST, LOW);
                GoSign = 0;
            }
            // 随意トリガ用パラメタの前回値を記憶
            TriggerLPF[0] = TriggerLPF[1];
            if(abs(S_MODE-preMODE)>0.9){
                EndTrigger++;
            }
            preMODE = S_MODE;
            delay(1);
        }
        delay(1);
    }
}
