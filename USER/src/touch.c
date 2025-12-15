#include "touch.h"
#include "xpt2046.h"
#include "lcd.h"
#include "led.h"
#include "common.h"
#include "key.h"
#include "uart.h"       // UART 串口驱动 (USART1/UART4)
#include "stdlib.h" // 用于 atof()
#include "AD9833.h"     // AD9833 DDS 信号发生器驱动
#include "dac7612.h"		// 程控比例放大
#include "learning_mode.h" // <<< 添加此 #include
#include "touch.h"
#include "xpt2046.h"
#include "lcd.h"
#include "led.h"
#include "common.h"
#include "key.h"
#include "uart.h"       // UART 串口驱动 (USART1/UART4)
#include "stdlib.h" // 用于 atof()
#include "AD9833.h"     // AD9833 DDS 信号发生器驱动
#include "dac7612.h"		// 程控比例放大
#include "learning_mode.h" // <<< 添加此 #include


/*********************************************************************************
*********************启明欣欣 STM32F407应用开发板(高配版)*************************
**********************************************************************************
* 文件名称: touch.c                                                              *
* 文件简述：触摸屏测试程序                                                       *
* 创建日期：2018.08.30                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：                                                                     * 
**********************************************************************************
*********************************************************************************/

//触摸屏初始化
void Touch_Init(void)
{
	if(lcd_id==0x9341)
	{
	  XPT2046_Init();
	}

}

//清空屏幕
void Clear_Screen(void)
{
	LCD_Clear(WHITE);//清屏   
 	BRUSH_COLOR=BLUE;//设置字体为蓝色 
	LCD_DisplayString(lcd_width-40,lcd_height-18,16,"Clear");//显示清屏区域
  BRUSH_COLOR=RED;//设置画笔蓝色 
}

/****************************************************************************
* 名    称: u8 Draw_Point(u16 x,u16 y,u16 color)
* 功    能：画一个点(4*4的点)	
* 入口参数：x,y:坐标
            color:颜色
* 返回参数: 无  
* 说    明：        
****************************************************************************/
void Draw_Point(u16 x,u16 y,u16 color)
{	    	
	u8 i=0;

	BRUSH_COLOR=color;
	for(i=0;i<4;i++)
	{
	 LCD_DrawPoint(x,y+i);
	 LCD_DrawPoint(x+1,y+i);
	 LCD_DrawPoint(x+2,y+i);
	 LCD_DrawPoint(x+3,y+i);
	} 	  	
}	

//电阻触摸屏测试函数
void R_Touch_test(void)
{
	u8 i=0;	  
	while(1)
	{
		if(PEN) LED1=0;
		else LED1=1;
		XPT2046_Scan(0); 		 
	
		 	if(Xdown<lcd_width&&Ydown<lcd_height)
			{	
				if(Xdown>(lcd_width-40)&&Ydown>lcd_height-18)Clear_Screen();  //清空屏幕
				else Draw_Point(Xdown,Ydown,RED);		//画图	  			   
			} 
		LCD_DisplayNum(10,10,Xdown,5,16,1);
		LCD_DisplayNum(150,10,Ydown,5,16,1);
			//LCD_DisplayString_color(10,10,16,*Xdown,RED,WHITE);
    	Draw_Point(Xdown,Ydown,RED);
		//i++;
		//if(i%20==0)
		//	LED1=!LED1;
	}
}


/**
 * @brief   LCD上触屏交互
 * @hint 		屏幕为320*250
 */
u8 button_cnt = 4;
u8 button_startX[4] = {25,75,125,175};
u8 button_startY = 125;
u8 button_endX[4] = {65,115,165,215};
u8 button_endY = 145;
u8 button_centerX[4] = {45,95,145,195};
u8 button_centerY = 135;
u8 judge_point;
u8 flagForTouch = 0;

// 用于判断所按按钮，返回中心点横坐标
int judge_button(){
	u8 i;
	for(i=0;i<button_cnt;i++){
		if(Ydown > button_startY && Ydown < button_endY && Xdown>button_startX[i] && Xdown<button_endX[i]){
			return button_centerX[i];
		}
	}
}

void LCD_TOUCH_TEST(){
		//初始化，设定四个按钮
		// Clear_Screen();
		for(u8 i=0;i<button_cnt;i++){
		LCD_Color_DrawRectangle(button_startX[i],button_startY,button_endX[i],button_endY,BLACK);
		}
		//扫描屏幕
		XPT2046_Scan(0);
		//sprintf(String, "%d %d",Xdown,Ydown);
		//LCD_DisplayString(10, 100, 12, (u8*)String);
		
		// 判断落点与输出,此处DIY
		judge_point = judge_button();
		for(u8 i=0;i<button_cnt;i++){
			if(judge_point == button_centerX[i]){
				// 模拟按键，修改值
				flagForTouch = i;
				
				sprintf(String, "HI There");
				LCD_DisplayString(judge_point - 18, button_centerY + 10 , 12, (u8*)String);
				sprintf(String, "flag:%d",flagForTouch);
				LCD_DisplayString(judge_point - 18, button_centerY + 20 , 12, (u8*)String);
			}
		}
}


	/*================================================================================*/
/* 内部类型定义 (Internal Type Definitions)                                       */
/*================================================================================*/

// 数字小键盘按钮结构体
typedef struct {
    u16 x, y, w, h;
    char* label;
} NumpadButton;


/*================================================================================*/
/* 全局变量 (Global Variables)                                                    */
/*================================================================================*/

// --- 系统状态 ---
volatile SystemState g_current_state = STATE_PARAM_SETTINGS;
volatile u8 g_needs_redraw = 1; // 屏幕重绘标志

// --- 参数 ---
#define PARAM_COUNT 5
Parameter g_params[PARAM_COUNT];
int g_selected_param_index = 0;

// --- 数字小键盘输入缓冲 ---
char g_input_buffer[20] = "";
int g_input_buffer_pos = 0;

// --- 数字小键盘布局 ---
NumpadButton g_numpad_buttons[NUMPAD_ROWS][NUMPAD_COLS];
const char* numpad_labels[NUMPAD_ROWS][NUMPAD_COLS] = {
    {"7", "8", "9", "C"},
    {"4", "5", "6", "-"}, // 预留
    {"1", "2", "3", "."},
    {"0", "OK", " ", " "} // 合并的OK按钮
};

/*================================================================================*/
/* 内部函数前向声明 (Internal Function Forward Declarations)                      */
/*================================================================================*/

// --- 初始化 ---
static void ui_init_params(void);
static void ui_init_numpad(void);

// --- 绘图 ---
static void ui_draw_full_screen(void);
static void ui_draw_base_layout(void);
static void ui_draw_param_list(void);
static void ui_draw_status_bar(void);
static void ui_draw_param_settings_screen(void);
static void ui_draw_placeholder_screen(const char* title);
static void ui_draw_numpad(void);
static void ui_update_param_details(void);
static void ui_update_input_display(void);

// --- 输入处理 ---
static void ui_handle_touch(void);
static void ui_handle_param_list_touch(u16 x, u16 y);
static void ui_handle_numpad_touch(u16 x, u16 y);
static void ui_process_numpad_input(const char* input);
static void ui_handle_hardware_keys(void);
static void ui_draw_simulation_screen(void);

// --- 状态管理 ---
static void ui_set_state(SystemState new_state);

/*================================================================================*/
/* 公共接口函数实现 (Public API Implementation)                                 */
/*================================================================================*/

/**
 * @brief 初始化整个UI系统。
 */
void ui_init(void) {
    ui_init_params();
    ui_init_numpad();
}

/**
 * @brief UI的主循环函数，处理所有UI事件。
 */
void ui_run(void) {
    // 1. 首先处理非阻塞的硬件按键输入
    ui_handle_hardware_keys();

    // 2. 处理需要长时间运行的阻塞任务
    //    当状态为LEARNING_START时，执行一次学习序列，然后立即切换到DONE状态
    if (g_current_state == STATE_LEARNING_START) {
        // 执行耗时操作
        execute_learning_sequence();  
        // 操作完成后，立刻切换到“完成”状态，并请求重绘以显示结果
        ui_set_state(STATE_LEARNING_DONE);
    }

    // 3. 如果屏幕需要重绘，则调用绘图函数
    if (g_needs_redraw) {
        ui_draw_full_screen();
        g_needs_redraw = 0;
    }

    // 4. 处理非阻塞的、持续性的任务，例如触摸输入
    if (g_current_state == STATE_PARAM_SETTINGS) {
        ui_handle_touch();
    }
    
    delay_ms(20); // 防止CPU忙等
}

/*================================================================================*/
/* 内部初始化函数 (Internal Initialization Functions)                           */
/*================================================================================*/

/**
 * @brief 用默认值初始化所有用户可配置的参数。
 */
void ui_init_params() {
    // 参数 0
    strcpy(g_params[0].name, "SetFreq");
    g_params[0].value = 1;
    g_params[0].min_val = 0.1;
    g_params[0].max_val = 1100.0;
    strcpy(g_params[0].unit, "KHz");
    strcpy(g_params[0].description, "Set frequency");

    // 参数 1
    strcpy(g_params[1].name, "SetAmp");
    g_params[1].value = 2.0;
    g_params[1].min_val = 1.0;
    g_params[1].max_val = 3.0;
    strcpy(g_params[1].unit, "Vpp");
    strcpy(g_params[1].description, "Signal peak-to-peak value");

    // 参数 2
    strcpy(g_params[2].name, "AmpBias");
    g_params[2].value = 0;
    g_params[2].min_val = -100;
    g_params[2].max_val = 100;
    strcpy(g_params[2].unit, "div");
    strcpy(g_params[2].description, "Compensation amplitude bias");
 
		 // 参数 3
    strcpy(g_params[3].name, "SWEEP_START");
    g_params[3].value = 500.0f;
    g_params[3].min_val = 100.0f;
    g_params[3].max_val = 2000.0f;
    strcpy(g_params[3].unit, "Hz");
    strcpy(g_params[3].description, "Start frequency");
		
		// 参数 4
    strcpy(g_params[4].name, "SWEEP_END");
    g_params[4].value = 250000.0f;
    g_params[4].min_val = 10000.0f;
    g_params[4].max_val = 1000000.0f;
    strcpy(g_params[4].unit, "Hz");
    strcpy(g_params[4].description, "Start frequency");
}

/**
 * @brief 初始化数字小键盘按钮的坐标和标签。
 */
void ui_init_numpad() {
    for (int r = 0; r < NUMPAD_ROWS; r++) {
        for (int c = 0; c < NUMPAD_COLS; c++) {
            g_numpad_buttons[r][c].x = NUMPAD_START_X + c * (NUMPAD_BTN_WIDTH + 5);
            g_numpad_buttons[r][c].y = NUMPAD_START_Y + r * (NUMPAD_BTN_HEIGHT + 5);
            g_numpad_buttons[r][c].w = NUMPAD_BTN_WIDTH;
            g_numpad_buttons[r][c].h = NUMPAD_BTN_HEIGHT;
            g_numpad_buttons[r][c].label = (char*)numpad_labels[r][c];
        }
    }
    // “OK”按钮的特殊处理，使其跨越两列
    g_numpad_buttons[3][1].w = NUMPAD_BTN_WIDTH * 2 + 5; 
}


/*================================================================================*/
/* 内部绘图函数 (Internal Drawing Functions)                                      */
/*================================================================================*/


/**
 * @brief 根据当前状态重绘整个屏幕。
 */
void ui_draw_full_screen() {
    // 仅在必要时才清屏
    if (g_current_state != STATE_LEARNING_DONE) {
         LCD_Clear(UI_BG_COLOR);
    }
   
    switch (g_current_state) {
        case STATE_PARAM_SETTINGS:
            ui_draw_base_layout();
            ui_draw_param_settings_screen();
            break;

        case STATE_DDS_OUTPUT:
            ui_draw_base_layout();
            ui_draw_placeholder_screen("DDS Output Mode");
            break;

        case STATE_LEARNING_START:
            // 在耗时任务执行前，可以显示一个“正在学习...”的提示
            ui_draw_placeholder_screen("Learning in progress...");
            break;
            
        case STATE_LEARNING_DONE:
            // 学习完成后，调用专用的结果绘制函数
            draw_learning_results();
            break;
            
        case STATE_SIMULATION:
            ui_draw_base_layout();
            ui_draw_simulation_screen(); // 调用新函数来显示系数
            break;
    }
    
    // 仅在非学习结果界面绘制底部状态栏
    if (g_current_state != STATE_LEARNING_DONE) {
       ui_draw_status_bar();
    }
}


/**
 * @brief 绘制静态布局元素（分割线、参数列表背景等）。
 */
void ui_draw_base_layout() {
    // 绘制垂直分割线
    LCD_DrawLine(PARAM_LIST_START_X, 0, PARAM_LIST_START_X, MAIN_AREA_HEIGHT);
    // 绘制参数列表
    ui_draw_param_list();
}

/**
 * @brief 在屏幕右侧绘制参数列表。
 */
void ui_draw_param_list() {
    u16 item_height = MAIN_AREA_HEIGHT / PARAM_COUNT;
    for (int i = 0; i < PARAM_COUNT; i++) {
        u16 start_y = i * item_height;
        if (i == g_selected_param_index) {
            // 高亮显示选中的参数
            LCD_Fill_onecolor(PARAM_LIST_START_X + 1, start_y, SCREEN_WIDTH, start_y + item_height - 1, UI_HIGHLIGHT_COLOR);
            LCD_DisplayString_color(PARAM_LIST_START_X + 5, start_y + item_height / 2 - 8, 16, (u8*)g_params[i].name, UI_HIGHLIGHT_TEXT_COLOR, UI_HIGHLIGHT_COLOR);
        } else {
            // 绘制未选中的参数
            LCD_Fill_onecolor(PARAM_LIST_START_X + 1, start_y, SCREEN_WIDTH, start_y + item_height - 1, UI_BG_COLOR);
            LCD_DisplayString_color(PARAM_LIST_START_X + 5, start_y + item_height / 2 - 8, 16, (u8*)g_params[i].name, UI_TEXT_COLOR, UI_BG_COLOR);
        }
        // 绘制列表项之间的分隔线
        LCD_DrawLine(PARAM_LIST_START_X, start_y + item_height -1, SCREEN_WIDTH, start_y + item_height -1);
    }
}


/**
 * @brief   绘制仿真模式界面，用于显示IIR滤波器系数。
 * @note    此函数会清空屏幕左侧主区域并显示所有系数值。
 */
void ui_draw_simulation_screen(void) 
{
    char buffer[50]; // 用于格式化字符串的缓冲区

    // 1. 清空主工作区 (屏幕左侧)
    LCD_Fill_onecolor(0, 0, PARAM_LIST_START_X - 1, MAIN_AREA_HEIGHT, UI_BG_COLOR);

    // 2. 绘制标题
    LCD_DisplayString_color(10, 10, 16, (u8*)"仿真模式 - IIR系数", UI_TEXT_COLOR, UI_BG_COLOR);

    // 3. 逐个显示 g_iir_coeffs 的参数
    //    确保 g_iir_coeffs 变量在其他文件中已正确定义并赋值
    
    // 显示 'b' 系数 (前馈系数)
    sprintf(buffer, "b0: %.6f", g_iir_coeffs.b[0]);
    LCD_DisplayString(20, 60, 16, (u8*)buffer);

    sprintf(buffer, "b1: %.6f", g_iir_coeffs.b[1]);
    LCD_DisplayString(20, 85, 16, (u8*)buffer);

    sprintf(buffer, "b2: %.6f", g_iir_coeffs.b[2]);
    LCD_DisplayString(20, 110, 16, (u8*)buffer);
    
    // 显示 'a' 系数 (反馈系数)
    sprintf(buffer, "a1: %.6f", g_iir_coeffs.a[1]);
    LCD_DisplayString(20, 145, 16, (u8*)buffer); // 增加一点行距以区分a, b

    sprintf(buffer, "a2: %.6f", g_iir_coeffs.a[2]);
    LCD_DisplayString(20, 170, 16, (u8*)buffer);
}

/**
 * @brief 在屏幕底部绘制状态栏。
 */
void ui_draw_status_bar() {
    char status_text[50];
    const char* state_str = "";

    // 使用新的状态定义来更新要显示的文本
    switch (g_current_state) {
        case STATE_PARAM_SETTINGS: state_str = "Parameter Settings"; break;
        case STATE_DDS_OUTPUT:     state_str = "DDS Output";         break;
        
        
        case STATE_LEARNING_START: state_str = "Learning Start...";  break;
        case STATE_LEARNING_DONE:  state_str = "Learning Done";      break; 
        

        case STATE_SIMULATION:     state_str = "Simulation Start";   break;
    }

    sprintf(status_text, "FLAG %d: %s", (int)g_current_state, state_str);
    
    // 绘制状态栏背景
    LCD_Fill_onecolor(0, SCREEN_HEIGHT - STATUS_BAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT, UI_STATUS_BG_COLOR);
    LCD_DrawLine(0, SCREEN_HEIGHT - STATUS_BAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - STATUS_BAR_HEIGHT);

    // 显示状态文本
    LCD_DisplayString_color(10, SCREEN_HEIGHT - STATUS_BAR_HEIGHT + 5, 16, (u8*)status_text, UI_TEXT_COLOR, UI_STATUS_BG_COLOR);
}

/**
 * @brief 绘制参数设置状态下的特定UI。
 */
void ui_draw_param_settings_screen() {
    ui_update_param_details();
    ui_draw_numpad();
}

/**
 * @brief 为未实现的状态绘制一个带标题的占位屏幕。
 * @param title 要显示的标题。
 */
void ui_draw_placeholder_screen(const char* title) {
    LCD_Fill_onecolor(0, 0, PARAM_LIST_START_X - 1, MAIN_AREA_HEIGHT, UI_BG_COLOR);
    LCD_DisplayString(20, 150, 24, (u8*)title);
}

/**
 * @brief 在屏幕上绘制数字小键盘。
 */
void ui_draw_numpad() {
    for (int r = 0; r < NUMPAD_ROWS; r++) {
        for (int c = 0; c < NUMPAD_COLS; c++) {
            NumpadButton btn = g_numpad_buttons[r][c];
            if (strcmp(btn.label, " ") == 0) continue; // 跳过空按钮

            u16 color = UI_BUTTON_COLOR;
            if(strcmp(btn.label, "OK") == 0) color = UI_CONFIRM_BTN_COLOR;
            if(strcmp(btn.label, "C") == 0) color = UI_CLEAR_BTN_COLOR;

            LCD_Fill_onecolor(btn.x, btn.y, btn.x + btn.w, btn.y + btn.h, color);
            LCD_Color_DrawRectangle(btn.x, btn.y, btn.x + btn.w, btn.y + btn.h, UI_BORDER_COLOR);
            LCD_DisplayString_color(btn.x + btn.w / 2 - strlen(btn.label) * 4, btn.y + btn.h / 2 - 8, 16, (u8*)btn.label, UI_BUTTON_TEXT_COLOR, color);
        }
    }
}


/**
 * @brief 更新参数详情区域（左上角）。
 */
void ui_update_param_details() {
    char buffer[100];
    Parameter p = g_params[g_selected_param_index];

    // 首先清空该区域
    LCD_Fill_onecolor(0, 0, PARAM_LIST_START_X - 1, NUMPAD_START_Y - 1, UI_BG_COLOR);

    // 参数名称
    sprintf(buffer, "Param: %s", p.name);
    LCD_DisplayString(5, 5, 16, (u8*)buffer);

    // 当前值
    sprintf(buffer, "Value: %.4f %s", p.value, p.unit);
    LCD_DisplayString(5, 25, 16, (u8*)buffer);

    // 上下限
    sprintf(buffer, "Range: %.2f to %.2f %s", p.min_val, p.max_val, p.unit);
    LCD_DisplayString(5, 45, 12, (u8*)buffer);

    // 描述
    LCD_DisplayString(5, 60, 12, (u8*)p.description);

    // 输入区域
    LCD_DrawRectangle(5, 85, PARAM_LIST_START_X - 5, 105);
    ui_update_input_display();
}

/**
 * @brief 更新正在输入框中键入的文本。
 */
void ui_update_input_display() {
    char display_buf[21];
    // 左对齐并用空格填充以清除旧文本
    sprintf(display_buf, "%-19s", g_input_buffer); 

    LCD_DisplayString_color(10, 90, 16, (u8*)display_buf, UI_TEXT_COLOR, UI_BG_COLOR);
}


/*================================================================================*/
/* 内部输入处理函数 (Internal Input Handling Functions)                           */
/*================================================================================*/

/**
 * @brief 主触摸处理函数，根据触摸位置委托给其他处理函数。
 */
void ui_handle_touch() {
    if (PEN == 0) { // 如果屏幕被按下
        delay_ms(10); // 消抖
        if (PEN == 0) {
            XPT2046_Scan(0); // 获取坐标 (竖屏模式下，参数为0)
            if (Xdown < SCREEN_WIDTH && Ydown < SCREEN_HEIGHT) {
                if (Xdown >= PARAM_LIST_START_X) {
                    ui_handle_param_list_touch(Xdown, Ydown);
                } else {
                    if (g_current_state == STATE_PARAM_SETTINGS) {
                        ui_handle_numpad_touch(Xdown, Ydown);
                    }
                }
            }
            while(PEN == 0); // 等待按键释放
        }
    }
}

/**
 * @brief 处理右侧参数列表上的触摸事件。
 */
void ui_handle_param_list_touch(u16 x, u16 y) {
    u16 item_height = MAIN_AREA_HEIGHT / PARAM_COUNT;
    int touched_index = y / item_height;

    if (touched_index >= 0 && touched_index < PARAM_COUNT) {
        if (g_selected_param_index != touched_index) {
            g_selected_param_index = touched_index;
            g_input_buffer_pos = 0;
            g_input_buffer[0] = '\0';
            
            // 重绘受影响的部分
            ui_draw_param_list();
            if (g_current_state == STATE_PARAM_SETTINGS) {
                ui_update_param_details();
            }
        }
    }
}

/**
 * @brief 处理数字小键盘上的触摸事件。
 */
void ui_handle_numpad_touch(u16 x, u16 y) {
    for (int r = 0; r < NUMPAD_ROWS; r++) {
        for (int c = 0; c < NUMPAD_COLS; c++) {
            NumpadButton btn = g_numpad_buttons[r][c];
            if (strcmp(btn.label, " ") == 0) continue;

            if (x > btn.x && x < (btn.x + btn.w) && y > btn.y && y < (btn.y + btn.h)) {
                // 按下按钮的视觉反馈
                LCD_Fill_onecolor(btn.x, btn.y, btn.x + btn.w, btn.y + btn.h, UI_BUTTON_PRESS_COLOR);
                LCD_DisplayString_color(btn.x + btn.w / 2 - strlen(btn.label) * 4, btn.y + btn.h / 2 - 8, 16, (u8*)btn.label, UI_BUTTON_TEXT_COLOR, UI_BUTTON_PRESS_COLOR);
                delay_ms(100);

                // 处理输入
                ui_process_numpad_input(btn.label);

                // 将按钮重绘为原始状态
                u16 color = UI_BUTTON_COLOR;
                if(strcmp(btn.label, "OK") == 0) color = UI_CONFIRM_BTN_COLOR;
                if(strcmp(btn.label, "C") == 0) color = UI_CLEAR_BTN_COLOR;
                LCD_Fill_onecolor(btn.x, btn.y, btn.x + btn.w, btn.y + btn.h, color);
                LCD_DisplayString_color(btn.x + btn.w / 2 - strlen(btn.label) * 4, btn.y + btn.h / 2 - 8, 16, (u8*)btn.label, UI_BUTTON_TEXT_COLOR, color);
                
                return; // 找到第一个被按下的按钮后退出
            }
        }
    }
}


/**
 * @brief 处理来自数字小键盘按钮按下的输入。
 * @param input 被按下按钮的标签 (例如, "7", ".", "OK", "-")。
 */
void ui_process_numpad_input(const char* input) {
    if (strcmp(input, "OK") == 0) {
        if (g_input_buffer_pos > 0) {
            double new_val = atof(g_input_buffer);
            // 将值限制在最小/最大范围内
            if (new_val < g_params[g_selected_param_index].min_val) {
                new_val = g_params[g_selected_param_index].min_val;
            } else if (new_val > g_params[g_selected_param_index].max_val) {
                new_val = g_params[g_selected_param_index].max_val;
            }
            g_params[g_selected_param_index].value = new_val;
            
            // 清空缓冲区并更新显示
            g_input_buffer_pos = 0;
            g_input_buffer[0] = '\0';
            ui_update_param_details();
        }
    } else if (strcmp(input, "C") == 0) {
        // 清空缓冲区
        g_input_buffer_pos = 0;
        g_input_buffer[0] = '\0';
        ui_update_input_display();
    } 
    // *** 新增: 处理负号输入 ***
    else if (strcmp(input, "-") == 0) {
        // 仅当输入框为空时，才允许输入负号
        if (g_input_buffer_pos == 0) {
            g_input_buffer[g_input_buffer_pos++] = '-';
            g_input_buffer[g_input_buffer_pos] = '\0';
            ui_update_input_display();
        }
    }
    else { // 数字或 '.'
        if (g_input_buffer_pos < sizeof(g_input_buffer) - 1) {
            // 防止输入多个小数点
            if (strcmp(input, ".") == 0 && strchr(g_input_buffer, '.') != NULL) {
                return;
            }
            g_input_buffer[g_input_buffer_pos++] = *input;
            g_input_buffer[g_input_buffer_pos] = '\0';
            ui_update_input_display();
        }
    }
}





/*================================================================================*/
/* 内部状态管理函数 (Internal State Management Functions)                         */
/*================================================================================*/

/**
 * @brief 设置一个新的系统状态并标记需要重绘。
 * @param new_state 要切换到的状态。
 */
void ui_set_state(SystemState new_state) {
    if (g_current_state != new_state) {
        g_current_state = new_state;
        g_needs_redraw = 1;
    }
}



/*=核心函数=*/

// 定义数据表的维度
#define FREQ_POINTS 30
#define AMP_POINTS  11

// 1. 频率轴 (对应表的行) - 存储在Flash中
const uint16_t freq_axis[FREQ_POINTS] = {
    100, 200, 300, 400, 500, 600, 700, 800, 900, 1000,
    1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000,
    2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000
};

// 2. 幅值轴 (对应表的列) - 存储在Flash中
const uint16_t amp_axis[AMP_POINTS] = {
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20
};

// 3. 核心数据查找表 (m值) - 同样使用const存储在Flash中
const uint16_t lookup_table[FREQ_POINTS][AMP_POINTS] = {
	//   1.0   1.1   1.2   1.3   1.4   1.5   1.6   1.7   1.8   1.9  2.0 
    {  928,  955,  972,  988,  1003, 1022, 1030, 1046, 1062, 1074, 1081 }, // 100 Hz (+40)
    {  852,  878,  893,  913,  928,  953,  958,  971,  983,  998, 1010 }, // 200 Hz (-48)
    {  834,  859,  878,  897,  912,  923,  942,  956,  968,  977,  990 }, // 300 Hz (-70)
    {  836,  861,  880,  898,  913,  925,  943,  957,  970,  982,  990 }, // 400 Hz (-80)
    {  854,  880,  897,  916,  931,  944,  961,  975,  986, 1003, 1010 }, // 500 Hz (-80)
    {  865,  889,  907,  934,  949,  962,  979,  993,  997, 1011, 1024 }, // 600 Hz (-80)
    {  886,  910,  927,  944,  959,  972,  989, 1003, 1017, 1031, 1044 }, // 700 Hz (-80)
    {  896,  919,  937,  952,  967,  981,  997, 1011, 1026, 1040, 1052 }, // 800 Hz (-90)
    {  914,  937,  953,  969,  984,  998, 1014, 1028, 1043, 1057, 1069 }, // 900 Hz (-90)
    {  932,  955,  970,  985, 1000, 1015, 1030, 1045, 1060, 1074, 1085 }, // 1000 Hz (-90)
    {  946,  969,  985, 1001, 1016, 1031, 1046, 1060, 1074, 1088, 1099 }, // 1100 Hz (-90)
    {  961,  983, 1000, 1017, 1033, 1048, 1062, 1076, 1089, 1102, 1113 }, // 1200 Hz (-90)
    {  975,  997, 1016, 1033, 1049, 1064, 1079, 1092, 1103, 1116, 1128 }, // 1300 Hz (-90)
    {  990, 1011, 1031, 1049, 1066, 1081, 1095, 1108, 1118, 1130, 1142 }, // 1400 Hz (-90)
    { 1005, 1025, 1047, 1065, 1083, 1098, 1112, 1124, 1133, 1144, 1157 }, // 1500 Hz (-90)
    { 1014, 1034, 1056, 1074, 1092, 1107, 1121, 1133, 1142, 1153, 1166 }, // 1600 Hz (-90)
    { 1029, 1049, 1071, 1090, 1108, 1123, 1136, 1148, 1157, 1168, 1181 }, // 1700 Hz (-90)
    { 1042, 1062, 1083, 1103, 1121, 1135, 1148, 1160, 1170, 1181, 1194 }, // 1800 Hz (-90)
    { 1053, 1073, 1094, 1114, 1133, 1147, 1160, 1173, 1185, 1198, 1211 }, // 1900 Hz (-90)
    { 1068, 1088, 1109, 1128, 1143, 1152, 1170, 1183, 1195, 1208, 1221 }, // 2000 Hz (-90)
    { 1080, 1100, 1120, 1139, 1155, 1170, 1181, 1192, 1212, 1219, 1232 }, // 2100 Hz (-90)
    { 1091, 1111, 1131, 1148, 1168, 1180, 1192, 1202, 1224, 1231, 1244 }, // 2200 Hz (-90)
    { 1102, 1122, 1142, 1160, 1180, 1192, 1202, 1215, 1235, 1247, 1255 }, // 2300 Hz (-90)
    { 1113, 1133, 1152, 1171, 1188, 1200, 1210, 1225, 1248, 1255, 1266 }, // 2400 Hz (-90)
    { 1121, 1141, 1160, 1179, 1200, 1211, 1229, 1233, 1257, 1264, 1274 }, // 2500 Hz (-90)
    { 1132, 1152, 1171, 1191, 1210, 1222, 1232, 1244, 1265, 1276, 1286 }, // 2600 Hz (-90)
    { 1144, 1164, 1182, 1208, 1227, 1239, 1249, 1260, 1276, 1284, 1297 }, // 2700 Hz (-90)
    { 1155, 1175, 1193, 1219, 1239, 1250, 1260, 1271, 1283, 1296, 1309 }, // 2800 Hz (-90)
    { 1165, 1185, 1203, 1230, 1250, 1261, 1270, 1281, 1293, 1306, 1319 }, // 2900 Hz (-90)
    { 1165, 1190, 1207, 1235, 1255, 1261, 1275, 1286, 1298, 1311, 1324 }  // 3000 Hz (-90)
};



float get_parameter(float frequency, float amplitude) {
    uint8_t x_idx, y_idx; // 幅值和频率的索引

	
		// 信号发生模式
		if(amplitude > 20) return 1430; // 恒定 3Vpp
	
    // --- 1. 边界检查和处理 (Clamping) ---
    if (frequency <= freq_axis[0]) {
        frequency = freq_axis[0];
    } else if (frequency >= freq_axis[FREQ_POINTS - 1]) {
        frequency = freq_axis[FREQ_POINTS - 1];
    }

    if (amplitude <= amp_axis[0]) {
        amplitude = amp_axis[0];
    } else if (amplitude >= amp_axis[AMP_POINTS - 1]) {
        amplitude = amp_axis[AMP_POINTS - 1];
    }

    // --- 2. 查找索引 ---
    // 查找频率索引 (y_idx)
    for (y_idx = 0; y_idx < FREQ_POINTS - 1; y_idx++) {
        if (frequency < freq_axis[y_idx + 1]) {
            break;
        }
    }

    // 查找幅值索引 (x_idx)
    for (x_idx = 0; x_idx < AMP_POINTS - 1; x_idx++) {
        if (amplitude < amp_axis[x_idx + 1]) {
            break;
        }
    }
    
    // --- 3. 直接返回值 ---
    if ((frequency == freq_axis[y_idx]) && (amplitude == amp_axis[x_idx])) {
        return (float)lookup_table[y_idx][x_idx];
    }
}



float result_param;

/**
 * @brief   处理硬件按键按下事件以改变系统状态
 */
void ui_handle_hardware_keys() {
    key_scan(1);

    if (keydown_data == 0) {
        return;
    }

    // 1. 优先处理全局返回键 (HOME/KEY5)
    if (keydown_data == KEY5_DATA) {
        // 无论当前在哪个界面，都返回到参数设置界面
        ui_set_state(STATE_PARAM_SETTINGS);
				ADC_Cmd(ADC3, DISABLE);
    }
		else{
					// 2. 然后再处理其他按键
				if (keydown_data == KEY4_DATA) {
						if (g_current_state != STATE_DDS_OUTPUT) {
								result_param = get_parameter(g_params[0].value*1000.0f, g_params[1].value*10.0f);
								result_param = result_param + g_params[2].value;
								DAC7612_Write_CHA(result_param);
								AD9833_WaveOut(SIN_WAVE, g_params[0].value*1000.0f, 0, 1);
								ui_set_state(STATE_DDS_OUTPUT);
						}
				} 
				else if (keydown_data == KEY3_DATA) { // 学习模式
						// 只有在主界面才能启动学习
						if (g_current_state == STATE_PARAM_SETTINGS) {
								 ui_set_state(STATE_LEARNING_START); // 启动学习流程
								ADC_Cmd(ADC3, DISABLE);
							
						}
				} 
				else if (keydown_data == KEY2_DATA) {
						if (g_current_state != STATE_SIMULATION) {
								ui_set_state(STATE_SIMULATION);
								// 启动 ADC3
								ADC_Cmd(ADC3, ENABLE);
						}
				}
		}
    

    // 3. 处理完后，必须清零键值
    keydown_data = 0;
}