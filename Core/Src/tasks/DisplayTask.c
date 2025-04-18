#include "stdlib.h"
#include "cmsis_os2.h"  // Для CMSIS RTOS v2
#include "ssd1306.h"
#include "ssd1306_types.h"
#include "ssd1306_fonts.h"

extern osMutexId_t i2c_1MutexHandle;
extern osMessageQueueId_t displayQueueHandle;

void DisplayTask_Start(void *parameters) {
   struct DisplayMessage_t msg = {0};
   ssd1306_Init();
    for (;;) {
        // Отримуємо повідомлення з черги
        if (osMessageQueueGet(displayQueueHandle, &msg, NULL, osWaitForever) == osOK) {
            // Перевіряємо наявність мютексу
            if (osMutexAcquire(i2c_1MutexHandle, osWaitForever) == osOK) {
                ssd1306_Fill(Black);  // Очистити екран
                ssd1306_SetCursor(50, 0);  // Встановити курсор для першого рядка
                ssd1306_WriteString(msg.line1, Font_7x10, White);
            
                // Встановити курсор для другого рядка, враховуючи висоту шрифта
                ssd1306_SetCursor(50, 11);  // Почати з 18 пікселів для другого рядка
                ssd1306_WriteString(msg.line2, Font_7x10, White);
            
                // Оновити екран
                ssd1306_UpdateScreen();
            
                // Звільнити мютекс
                osMutexRelease(i2c_1MutexHandle);
            }
        }
    }
}
