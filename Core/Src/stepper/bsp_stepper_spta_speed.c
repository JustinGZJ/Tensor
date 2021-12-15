/**
  ******************************************************************************
  * @file    bsp_stepper_spta_speed.c
  * @author  justin
  * @version V1.0
  * @date    2021-12-11
  * @brief   步进电机SPTA加减速算法
  */
#include "bsp_stepper_spta_speed.h"
#include <stdlib.h>

#define max(a, b) (a>b?a:b)
#define min(a, b) (a<b?a:b)
static uint32_t cvtVelocity2Val(float value);
static uint32_t cvtVal2Velocity(float value);



SPTAData_Typedef spta_data = {0};

/**  @brief 启动spta加减速
  *  @param steps：移动的步数，不考虑细分的整步数。正数为顺时针，负数为逆时针。
  *  @param targetvelocity：，其值大小决定spta的速度曲线轮廓为三角轮最大速度限制廓或梯形轮廓
  *  @param accel：加速度数值
  */
void Stepper_Move_SPTA(int32_t steps, uint32_t targetvelocity, uint32_t accel) {
    SPTAData_Typedef *pspta = &spta_data;

    /* 判断是否正在加减速 */
    if (pspta->step_state != IDLE)
        return;

    /* 判断方向 */
    (steps < 0) ? MOTOR_DIR(CCW) : MOTOR_DIR(CW);
    steps = abs(steps);

    /* SPTA运行参数初始化 */
    pspta->steps_required = steps * MICRO_STEP;
    pspta->steps_middle = pspta->steps_required >> 1;

    pspta->TargetVelocity = targetvelocity;
    pspta->acceleration = accel;

    pspta->step_state = ACCELERATING;

    /* 启动定时器 */
    HAL_TIM_Base_Start_IT(&TIM_StepperHandle);
}


/**
  * @brief  spta速度决策
  * @param  *pspta：spta数据结构体指针
  * @retval 无
  */
void SPTA_Speed_Decision(SPTAData_Typedef *pspta) {

    if (pspta->step_mode == POSITION) {
        switch (pspta->step_state) {
            /* SPTA加速状态 */
            case ACCELERATING:
                /* 是否达到最大速度限制*/
                if (pspta->ActualVelocity >= pspta->TargetVelocity) {
                    /* 达到最大速度 */
                    pspta->ActualVelocity = pspta->TargetVelocity;
                    /* 此时走过的步数就是加速阶段的步数 */
                    pspta->steps_acced = pspta->steps_taken;
                    /* 切换成匀速状态 */
                    pspta->step_state = UNIFORM;
                    break;
                }

                /* 加速度值加到速度累加器 */
                pspta->VelAccumulator += pspta->acceleration;

                /* 速度累加器是否溢出 */
                if ((pspta->VelAccumulator >> 17) == 1) {
                    /* 溢出标志清零 */
                    pspta->VelAccumulator &= ~(1 << 17);
                    /* 加速阶段，速度累加器溢出，则速度值递增 */
                    pspta->ActualVelocity++;
                }

                if (pspta->steps_middle != 0) {
                    /* 已走过的步数≥中点步数，则开始减速 */
                    if (pspta->steps_taken >= pspta->steps_middle) {
                        pspta->step_state = DECELERATING;
                    }
                } else if (pspta->steps_taken > 0) {
                    /* 只运行一步，直接变减速 */
                    pspta->step_state = DECELERATING;
                }
                break;
                /* SPTA匀速状态 */
            case UNIFORM:
                /* 匀速阶段，剩余步数≤加速阶段步数，则开始减速 */
                if ((pspta->steps_required - pspta->steps_taken) <= pspta->steps_acced) {
                    pspta->step_state = DECELERATING;
                }
                break;
                /* SPTA减速状态 */
            case DECELERATING:
                /* 到达设定的步数之后停止运行 */
                if (pspta->steps_taken >= pspta->steps_required) {
                    /* 切换成空闲状态 */
                    pspta->step_state = IDLE;
                    break;
                }

                /* 加速度值加到速度累加器 */
                pspta->VelAccumulator += pspta->acceleration;
                /* 速度累加器是否溢出 */
                if ((pspta->VelAccumulator >> 17) == 1) {
                    /* 溢出标志清零 */
                    pspta->VelAccumulator &= ~(1 << 17);
                    /* 减速阶段，速度累加器溢出，速度值递减 */
                    pspta->ActualVelocity--;
                }
                break;
                /* SPTA空闲状态 */
            case IDLE:
                /* 清零spta相关数据 */
                pspta->TargetVelocity = 0;
                pspta->step_accumulator = 0;
                pspta->VelAccumulator = 0;
                pspta->steps_acced = 0;
                pspta->ActualVelocity = 0;
                pspta->steps_taken = 0;
                /* 关闭定时器 */
                HAL_TIM_Base_Stop_IT(&TIM_StepperHandle);
                break;
            default:
                break;
        }
    } else if (pspta->step_mode == VELOCITY) {
        if (pspta->ActualVelocity != pspta->TargetVelocity) //如果实际速度！=目标速度
        {
            pspta->VelAccumulator += pspta->acceleration;
            /* 速度累加器是否溢出 */
            if ((pspta->VelAccumulator >> 17) == 1) {
                /* 溢出标志清零 */
                pspta->VelAccumulator &= ~(1 << 17);
                /* 加速阶段，速度累加器溢出，则速度值递增 */
                //  pspta->ActualVelocity++;
                if (pspta->ActualVelocity < pspta->TargetVelocity)  //如果实际速度<目标速度
                {
                    pspta->ActualVelocity = min(pspta->ActualVelocity + 1, pspta->TargetVelocity); //实际速度为两者中小者
                } else if (pspta->ActualVelocity > pspta->TargetVelocity) //如果实际速度>大于目标速度
                {
                    pspta->ActualVelocity = max(pspta->ActualVelocity - 1, pspta->TargetVelocity); //实际速度为两者中大者
                }
            }
        }
    }

    pspta->step_accumulator += pspta->ActualVelocity;
    pspta->half_step_accumulator += pspta->ActualVelocity;
    if (pspta->half_step_accumulator >> 16 == 1) {
        pspta->half_step_accumulator &= ~(1 << 16);
        MOTOR_PUL(HIGH);
    }
    /* 步数累加器是否溢出 */
    if ((pspta->step_accumulator >> 17) == 1) {
        /* 溢出标志清零 */
        pspta->step_accumulator &= ~(1 << 17);
        /* 已走步数+1 */
        pspta->steps_taken++;
        MOTOR_PUL(LOW);
        /* 拉高脉冲引脚产生一个步进脉冲 */
    }
}

/**
  * @brief  将速度值转换为装载值.
  * @param  value:   circle/s
  * @retval 速度装载值
  */
static uint32_t cvtVelocity2Val(float value)
{
    return 1.0*value*(1<<17)/MOTOR_TIM_FREQ+1;
}
/**
  * @brief  将装载值转换为速度值.
  * @param  value:   pulse
  * @retval
  */
static uint32_t cvtVal2Velocity(float value)
{
    return 1.0*MOTOR_TIM_FREQ*value/((1<<17));
}
/**
  * @brief  设置为速度模式.
  * @param  pspta:参数
  * @param  speed:速度
  * @retval 无
  */
void SPTA_SetSpeed(SPTAData_Typedef *pspta, uint32_t speed,uint32_t accel)
{
    pspta->step_mode=VELOCITY;
    pspta->TargetVelocity = cvtVelocity2Val(speed);
    pspta->acceleration= cvtVelocity2Val(accel);

    /* 启动定时器 */
    HAL_TIM_Base_Start_IT(&TIM_StepperHandle);
}



void MOTOR_TIM_IRQHandler() {
    if (__HAL_TIM_GET_FLAG(&TIM_StepperHandle, TIM_FLAG_UPDATE)) {
        SPTA_Speed_Decision(&spta_data);
        __HAL_TIM_CLEAR_FLAG(&TIM_StepperHandle, TIM_FLAG_UPDATE);
    }
//  HAL_TIM_IRQHandler(&TIM_StepperHandle);
}
