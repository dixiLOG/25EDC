#ifndef __TOUCH_H__
#define __TOUCH_H__
#include "common.h"

void Touch_Init(void);
void Clear_Screen(void);
void Draw_Point(u16 x,u16 y,u16 color);	//在按下处画一个2*2点		
void R_Touch_test(void);
void LCD_TOUCH_TEST(void);

#include "stm32f4xx.h"

/*================================================================================*/
/* UI 配置与宏定义 (UI Configuration & Macros)                                   */
/*================================================================================*/

// 屏幕尺寸 
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320

// UI 颜色定义
#define UI_BG_COLOR             WHITE
#define UI_TEXT_COLOR           BLACK
#define UI_BORDER_COLOR         GRAY
#define UI_HIGHLIGHT_COLOR      BLUE
#define UI_HIGHLIGHT_TEXT_COLOR WHITE
#define UI_BUTTON_COLOR         0xBDF7 // 浅灰色
#define UI_BUTTON_PRESS_COLOR   0x7BEF // 深灰色
#define UI_BUTTON_TEXT_COLOR    BLACK
#define UI_STATUS_BG_COLOR      0xCE59 // 浅蓝色
#define UI_CONFIRM_BTN_COLOR    0x5E9B // 绿色
#define UI_CLEAR_BTN_COLOR      0xFACD // 橙色

// 布局定义
#define PARAM_LIST_START_X      (SCREEN_WIDTH * 3 / 4) // -> 180px
#define STATUS_BAR_HEIGHT       25
#define MAIN_AREA_HEIGHT        (SCREEN_HEIGHT - STATUS_BAR_HEIGHT) // -> 295px

// 数字小键盘按钮定义 
#define NUMPAD_COLS 4
#define NUMPAD_ROWS 4
#define NUMPAD_BTN_WIDTH 35
#define NUMPAD_BTN_HEIGHT 40
#define NUMPAD_START_X 12
#define NUMPAD_START_Y 120

/*================================================================================*/
/* 类型定义 (Type Definitions)                                                    */
/*================================================================================*/

// 系统状态枚举
typedef enum {
    STATE_PARAM_SETTINGS = 0,
    STATE_DDS_OUTPUT     = 1,
		// --- 新的状态定义 ---
    STATE_LEARNING_START,   // 触发学习模式的启动信号
    STATE_LEARNING_DONE,    // 学习完成，显示结果并等待按键
    // -------------------
    STATE_SIMULATION     = 4
} SystemState;

// 参数结构体
typedef struct {
    char name[12];
    float value;
    float min_val;
    float max_val;
    char unit[6];
    char description[50];
} Parameter;


/*================================================================================*/
/* 外部函数声明 (External Function Prototypes)                                    */
/*================================================================================*/

#define PARAM_COUNT 5

extern Parameter g_params[PARAM_COUNT];

/**
 * @brief 初始化整个UI系统。
 * @note  此函数应在系统和LCD初始化后调用。
 */
void ui_init(void);

/**
 * @brief UI的主循环函数。
 * @note  此函数应在main函数的while(1)循环中被连续调用。
 * 它负责处理按键、触摸和屏幕重绘。
 */
void ui_run(void);



#endif // __UI_H__
