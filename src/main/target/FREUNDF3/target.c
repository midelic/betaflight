/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM15, CH1, PA2,  TIM_USE_MOTOR,               1), // PWM1 - 
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,               1), // PWM2 - 
    DEF_TIM(TIM3,  CH4, PB7,  TIM_USE_MOTOR,               1), // PWM3 - 
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MOTOR,               1), // PWM4 - 
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_MOTOR | TIM_USE_LED, 1), // PWM5 - 
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_MOTOR,               1), // PWM6 - 
};
